/* Copyright (c) 2010-2017. The SimGrid Team. All rights reserved.          */

/* This program is free software; you can redistribute it and/or modify it
 * under the terms of the license (GNU LGPL) which comes with this package. */

#include "simgrid/plugins/temperature.h"
#include "simgrid/plugins/energy.h"
#include "simgrid/simix.hpp"
#include "src/plugins/vm/VirtualMachineImpl.hpp"
#include "src/surf/cpu_interface.hpp"

#include "simgrid/s4u/Engine.hpp"

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <string>
#include <utility>
#include <vector>

/** @addtogroup plugin_temperature

This is the temperature plugin, enabling to account not only for computation time, but also for the temperature in the
simulated platform.
To activate this plugin, first call sg_host_temperature_plugin_init() before your #MSG_init(), and then use
MSG_host_get_consumed_temperature() to retrieve the temperature of a given host.

When the host is on, this temperature naturally depends on the energy consumed by the processor, calculated by the energy plugin.
This model is based on the thermodynamics heat formula. Please consult <a href="http://www.softschools.com/formulas/physics/specific_heat_formula/61/">this page</a> to see more details.
TODO: This model is OK for a first iteration of the temperature plugin. More work to improve the model is necessary.

Here is an example of XML declaration:

\code{.xml}
<host id="MyHost1" speed="100.0Mf,50.0Mf,20.0Mf" pstate="0" core="4" >
      <prop id="watt_per_state" value="100.0:120.0:200.0, 93.0:110.0:170.0, 90.0:105.0:150.0" />
      <prop id="watt_off" value="10" />
      <prop id="specific_heat" value="0.39"/>
      <prop id="watt_cooling_pwr" value="50.0"/>
      <prop id="cooling_fluid_temperature" value="20.0"/>
      <prop id="grams_processor_mass" value="150"/>
    </host>
\endcode

This example gives the following parameters: Specific Heat of the processor is 0.39 Joules/(grams*kelvin); Cooling device power is 50 Watts;
Cooling fluid temperature is 20 Celcius; Processor mass is 150 grams.
This is enough to compute the temperature in function of the consumed energy


### How accurate are these models?

This model is at its first iteration. Further evaluation of its accuracy is needed. Please keep this in mind when using this plugin.
 */

XBT_LOG_NEW_DEFAULT_SUBCATEGORY(surf_temperature, surf, "Logging specific to the SURF temperature plugin");

namespace simgrid {
namespace temperature {

class HostTemperature {
public:
  static simgrid::xbt::Extension<simgrid::s4u::Host, HostTemperature> EXTENSION_ID;

  explicit HostTemperature(simgrid::s4u::Host* ptr);
  ~HostTemperature();

  double getTemperature();
  double getAmbientTemperature();
  void update();

private:
  simgrid::s4u::Host* host = nullptr;

public:
  /* Fluid temperature is assumed that each host is in a different place, that is,
   * a host does not influence in the fluid temperature of another host. Default value
   * is related to the normal temperature and pressure standard (NTP)
   * The fluid can be assumed as water in a water cooling system or the air temperature inside the node
   * TODO: The fluid temperature is assumed to be constant at the whole simulation. This can be reasonable in
   * the case of water cooling or nodes inside a temperature controlled data center, but not in the case of hosts
   * being used as radiators
   */
  double fluid_temperature = 20.0;/*< Temperature of the cooling fluid (in celsius) where the host is located >*/
  double temperature = fluid_temperature; /*< Current temperature (in celcius) of the host >*/
  double cooling_power = 50.0; /*< cooling power of the cooling device in watt=joules/second >*/
  double specific_heat = 0.39; /*< specific heat of the processor, defaulted to the value of copper in joules/(grams*kelvin)>*/
  double processor_mass = 150; /*< processor mass in grams >*/
  double last_energy;
  double consumed_energy_at_period;
  double last_temperature = fluid_temperature;
  double last_updated;       /*< Timestamp of the last temperature update event*/
};

simgrid::xbt::Extension<simgrid::s4u::Host, HostTemperature> HostTemperature::EXTENSION_ID;

/* Computes the temperature. Called lazily on need. */
void HostTemperature::update()
{
  if (dynamic_cast<simgrid::s4u::VirtualMachine*>(host)) // Ignore virtual machines
      return;
  double start_time  = this->last_updated;
  double finish_time = surf_get_clock();
  double current_speed = host->getSpeed();

  if (start_time < finish_time) {

    /*TODO: all the code below is a rough heating model!
     * This model is based on the very simple thermodynamics equation
     * energy = mass * specific_heat * (new_temperature - last_temperature)
     */
    this->consumed_energy_at_period = sg_host_get_consumed_energy(host) - this->last_energy; /*< Energy consumed in the time period >*/

    /*
     * Calculate the energy removed from the processor in the time period
     */
    double elapsed_time = finish_time - start_time;
    double dissipated_energy_at_period = this->cooling_power * elapsed_time;

    /*
     * Calculate the difference of energy in the time period
     */
    double energy_transfer_difference = consumed_energy_at_period - dissipated_energy_at_period;

    /*
     * Calculate the temperature change. It can be positive (increase) or negative (decrease)
     * temperature change is in Kelvin, this difference is the same for Celcius
     */
    double temperature_change = energy_transfer_difference / (this->processor_mass * this->specific_heat);
    double new_temperature = temperature_change + this->last_temperature;


    /*
     * Not so good workaround to handle temperature equilibrium between the processor and the cooling fluid
     */
     this->temperature = new_temperature >= this->fluid_temperature ? new_temperature : this->fluid_temperature;

    XBT_DEBUG("[update_temperature of %s] period=[%.2f-%.2f]; current power peak=%.0E flop/s; current_temperature: %.2f Celcius -> "
              "%.2f Celcius",
              host->getCname(), start_time, finish_time, host->pimpl_cpu->speed_.peak, this->last_temperature,
              this->temperature);

    this->last_updated = surf_get_clock();
    this->last_energy = sg_host_get_consumed_energy(host);
    this->last_temperature = this->temperature;
  }
}

HostTemperature::HostTemperature(simgrid::s4u::Host* ptr) : host(ptr), last_updated(surf_get_clock())
{
  this->last_energy = sg_host_get_consumed_energy(host);
  this->consumed_energy_at_period = this->last_energy;

  /*
   * Read some properties from the platform xml file
   */

  /*
   * specific_heat
   */
  const char* sp_heat_str = host->getProperty("specific_heat");
  if (sp_heat_str != nullptr) {
      try {
        this->specific_heat = std::stof(std::string(sp_heat_str));
      } catch (std::invalid_argument& ia) {
        throw std::invalid_argument(std::string("Invalid value for property specific_heat of host ") + host->getCname() +
                                    ": " + sp_heat_str);
      }
    } else{
	  XBT_WARN("Missing value for property specific_heat of host %s: Using default value %.2f", host->getCname(),
			  this->specific_heat);
    }

    /*
     * cooling_power
     */
    const char* cool_power_str = host->getProperty("watt_cooling_pwr");
    if (cool_power_str != nullptr) {
        try {
          this->cooling_power = std::stof(std::string(cool_power_str));
        } catch (std::invalid_argument& ia) {
          throw std::invalid_argument(std::string("Invalid value for property watt_cooling_pwr of host ") + host->getCname() +
                                      ": " + cool_power_str);
        }
      } else{
  	  XBT_WARN("Missing value for property watt_cooling_pwr of host %s: Using default value %.2f watts", host->getCname(),
  			  this->cooling_power);
      }

      /*
       * fluid_temperature
       */
      const char* fluid_temp_str = host->getProperty("cooling_fluid_temperature");
      if (fluid_temp_str != nullptr) {
          try {
            this->fluid_temperature = std::stof(std::string(fluid_temp_str));
          } catch (std::invalid_argument& ia) {
            throw std::invalid_argument(std::string("Invalid value for property cooling_fluid_temperature of host ") + host->getCname() +
                                        ": " + fluid_temp_str);
          }
        } else{
    	  XBT_WARN("Missing value for property cooling_fluid_temperature of host %s: Using default value %.2f Celcius (NTP standard)", host->getCname(),
    			  this->fluid_temperature);
        }

        /*
         * processor_mass
         */
        const char* proc_mass_str = host->getProperty("grams_processor_mass");
        if (proc_mass_str != nullptr) {
            try {
              this->processor_mass = std::stof(std::string(proc_mass_str));
            } catch (std::invalid_argument& ia) {
              throw std::invalid_argument(std::string("Invalid value for property grams_processor_mass of host ") + host->getCname() +
                                          ": " + proc_mass_str);
            }
          } else{
      	  XBT_WARN("Missing value for property grams_processor_mass of host %s: Using default value %.2f grams", host->getCname(),
      			  this->processor_mass);
          }

}

HostTemperature::~HostTemperature() = default;

double HostTemperature::getTemperature()
{
  if (last_updated < surf_get_clock()) // We need to simcall this as it modifies the environment
    simgrid::simix::kernelImmediate(std::bind(&HostTemperature::update, this));

  return temperature;
}

double HostTemperature::getAmbientTemperature()
{
  if (last_updated < surf_get_clock()) // We need to simcall this as it modifies the environment
    simgrid::simix::kernelImmediate(std::bind(&HostTemperature::update, this));

  return fluid_temperature;
}
}
}

using simgrid::temperature::HostTemperature;

/* **************************** events  callback *************************** */
static void onCreation(simgrid::s4u::Host& host)
{
  if (dynamic_cast<simgrid::s4u::VirtualMachine*>(&host)) // Ignore virtual machines
    return;

  // TODO Trace: set to zero the temperature variable associated to host->getName()

  host.extension_set(new HostTemperature(&host));
}

static void onActionStateChange(simgrid::surf::CpuAction* action, simgrid::surf::Action::State previous)
{
  for (simgrid::surf::Cpu* const& cpu : action->cpus()) {
    simgrid::s4u::Host* host = cpu->getHost();
    if (host != nullptr) {

      // If it's a VM, take the corresponding PM
      simgrid::s4u::VirtualMachine* vm = dynamic_cast<simgrid::s4u::VirtualMachine*>(host);
      if (vm) // If it's a VM, take the corresponding PM
        host = vm->pimpl_vm_->getPm();

      // Get the host_temperature extension for the relevant host
      HostTemperature* host_temperature = host->extension<HostTemperature>();

      if (host_temperature->last_updated < surf_get_clock())
        host_temperature->update();
    }
  }
}

/* This callback is fired either when the host changes its state (on/off) ("onStateChange") or its speed
 * (because the user changed the pstate, or because of external trace events) ("onSpeedChange") */
static void onHostChange(simgrid::s4u::Host& host)
{
 if (dynamic_cast<simgrid::s4u::VirtualMachine*>(&host)) // Ignore virtual machines
    return;

  HostTemperature* host_temperature = host.extension<HostTemperature>();

  host_temperature->update();
}

static void onHostDestruction(simgrid::s4u::Host& host)
{
  if (dynamic_cast<simgrid::s4u::VirtualMachine*>(&host)) // Ignore virtual machines
    return;

  HostTemperature* host_temperature = host.extension<HostTemperature>();
  host_temperature->update();
  XBT_INFO("Temperature of host %s: %f Celcius", host.getCname(), host_temperature->getTemperature());
}

static void onSimulationEnd()
{
  /*TODO: Do nothing. Maybe do something in another version
   */
  return;
}

/* **************************** Public interface *************************** */
extern "C" {

/** \ingroup plugin_temperature
 * \brief Enable host temperature plugin
 * \details Enable temperature plugin to get the temperature of each cpu. Call this function before #MSG_init().
 */
void sg_host_temperature_plugin_init()
{
  if (HostTemperature::EXTENSION_ID.valid())
    return;
  sg_host_energy_plugin_init();
  HostTemperature::EXTENSION_ID = simgrid::s4u::Host::extension_create<HostTemperature>();

  simgrid::s4u::Host::onCreation.connect(&onCreation);
  simgrid::s4u::Host::onStateChange.connect(&onHostChange);
  simgrid::s4u::Host::onSpeedChange.connect(&onHostChange);
  simgrid::s4u::Host::onDestruction.connect(&onHostDestruction);
  simgrid::s4u::onSimulationEnd.connect(&onSimulationEnd);
  simgrid::surf::CpuAction::onStateChange.connect(&onActionStateChange);
}

/** @ingroup plugin_temperature
 *  @brief updates the temperature of all hosts
 *
 * After this call, sg_host_get_temperature() will not interrupt your process
 * (until after the next clock update).
 */
void sg_host_temperature_update_all()
{
  simgrid::simix::kernelImmediate([]() {
    std::vector<simgrid::s4u::Host*> list;
    simgrid::s4u::Engine::getInstance()->getHostList(&list);
    for (auto const& host : list)
      if (dynamic_cast<simgrid::s4u::VirtualMachine*>(host) == nullptr) // Ignore virtual machines
        host->extension<HostTemperature>()->update();
  });
}

/** @ingroup plugin_temperature
 *  @brief Returns the current temperature by the host (in Celcius)
 *
 *  Please note that since the temperature is lazily updated, it may require a simcall to update it.
 *  The result is that the actor requesting this value will be interrupted,
 *  the value will be updated in kernel mode before returning the control to the requesting actor.
 */
double sg_host_get_temperature(sg_host_t host)
{
  xbt_assert(HostTemperature::EXTENSION_ID.valid(),
             "The Temperature plugin is not active. Please call sg_temperature_plugin_init() during initialization.");
  return host->extension<HostTemperature>()->getTemperature();
}

/** @ingroup plugin_temperature
 *  @brief Returns the total temperature consumed by the host so far (in Celcius)
 *  TODO: THIS VALUE IS CURRENT CONSTANT. THE AMBIENT TEMPERATURE WILL BE COMPUTED DYNAMICALLY IN A FUTURE VERSION
 *  Please note that since the temperature is lazily updated, it may require a simcall to update it.
 *  The result is that the actor requesting this value will be interrupted,
 *  the value will be updated in kernel mode before returning the control to the requesting actor.
 */
double sg_host_get_ambient_temperature(sg_host_t host)
{
  xbt_assert(HostTemperature::EXTENSION_ID.valid(),
             "The Temperature plugin is not active. Please call sg_temperature_plugin_init() during initialization.");
  return host->extension<HostTemperature>()->getAmbientTemperature();
}
}
