/* Copyright (c) 2015-2017. The SimGrid Team. All rights reserved.          */

/* This program is free software; you can redistribute it and/or modify it
 * under the terms of the license (GNU LGPL) which comes with this package. */

#include "xbt/log.h"

#include "simgrid/s4u/File.hpp"
#include "simgrid/s4u/Host.hpp"
#include "simgrid/s4u/Storage.hpp"
#include "simgrid/simix.hpp"
#include "src/surf/HostImpl.hpp"

XBT_LOG_NEW_DEFAULT_CATEGORY(s4u_file,"S4U files");

namespace simgrid {
namespace s4u {

File::File(std::string fullpath, void* userdata) : File(fullpath, Host::current(), userdata){};

File::File(std::string fullpath, sg_host_t host, void* userdata) : fullpath_(fullpath), userdata_(userdata)
{
  // this cannot fail because we get a xbt_die if the mountpoint does not exist
  Storage* st                  = nullptr;
  size_t longest_prefix_length = 0;
  XBT_DEBUG("Search for storage name for '%s' on '%s'", fullpath.c_str(), host->getCname());

  for (auto const& mnt : host->getMountedStorages()) {
    XBT_DEBUG("See '%s'", mnt.first.c_str());
    mount_point_ = fullpath.substr(0, mnt.first.length());

    if (mount_point_ == mnt.first && mnt.first.length() > longest_prefix_length) {
      /* The current mount name is found in the full path and is bigger than the previous*/
      longest_prefix_length = mnt.first.length();
      st                    = mnt.second;
    }
  }
  if (longest_prefix_length > 0) { /* Mount point found, split fullpath into mount_name and path+filename*/
    mount_point_ = fullpath.substr(0, longest_prefix_length);
    path_        = fullpath.substr(longest_prefix_length, fullpath.length());
  } else
    xbt_die("Can't find mount point for '%s' on '%s'", fullpath.c_str(), host->getCname());

  localStorage = st;

  XBT_DEBUG("\tOpen file '%s'", path_.c_str());
  std::map<std::string, sg_size_t>* content = localStorage->getContent();
  // if file does not exist create an empty file
  auto sz = content->find(path_);
  if (sz != content->end()) {
    size_ = sz->second;
  } else {
    size_ = 0;
    content->insert({path_, size_});
    XBT_DEBUG("File '%s' was not found, file created.", path_.c_str());
  }
}

sg_size_t File::read(sg_size_t size)
{
  XBT_DEBUG("READ %s on disk '%s'", getPath(), localStorage->getCname());
  // if the current position is close to the end of the file, we may not be able to read the requested size
  sg_size_t read_size = localStorage->read(std::min(size, size_ - current_position_));
  current_position_ += read_size;
  return read_size;
}

sg_size_t File::write(sg_size_t size)
{
  XBT_DEBUG("WRITE %s on disk '%s'. size '%llu/%llu'", getPath(), localStorage->getCname(), size, size_);
  // If the storage is full before even starting to write
  if (localStorage->getSizeUsed() >= localStorage->getSize())
    return 0;
  /* Substract the part of the file that might disappear from the used sized on the storage element */
  localStorage->decrUsedSize(size_ - current_position_);

  sg_size_t write_size = localStorage->write(size);
  current_position_ += write_size;
  size_ = current_position_;

  localStorage->getContent()->erase(path_);
  localStorage->getContent()->insert({path_, size_});

  return write_size;
}

sg_size_t File::size()
{
  return size_;
}

void File::seek(sg_offset_t offset)
{
  current_position_ = offset;
}

void File::seek(sg_offset_t offset, int origin)
{
  switch (origin) {
    case SEEK_SET:
      current_position_ = offset;
      break;
    case SEEK_CUR:
      current_position_ += offset;
      break;
    case SEEK_END:
      current_position_ = size_ + offset;
      break;
    default:
      break;
  }
}

sg_size_t File::tell()
{
  return current_position_;
}

void File::move(std::string fullpath)
{
  /* Check if the new full path is on the same mount point */
  if (not strncmp(mount_point_.c_str(), fullpath.c_str(), mount_point_.length())) {
    std::map<std::string, sg_size_t>* content = localStorage->getContent();
    auto sz = content->find(path_);
    if (sz != content->end()) { // src file exists
      sg_size_t new_size = sz->second;
      content->erase(path_);
      std::string path = fullpath.substr(mount_point_.length(), fullpath.length());
      content->insert({path.c_str(), new_size});
      XBT_DEBUG("Move file from %s to %s, size '%llu'", path_.c_str(), fullpath.c_str(), new_size);
    } else {
      XBT_WARN("File %s doesn't exist", path_.c_str());
    }
  } else {
    XBT_WARN("New full path %s is not on the same mount point: %s.", fullpath.c_str(), mount_point_.c_str());
  }
}

int File::unlink()
{
  /* Check if the file is on local storage */
  if (localStorage->getContent()->find(path_) == localStorage->getContent()->end()) {
    XBT_WARN("File %s is not on disk %s. Impossible to unlink", path_.c_str(), localStorage->getCname());
    return -1;
  } else {
    XBT_DEBUG("UNLINK %s on disk '%s'", path_.c_str(), localStorage->getCname());
    localStorage->decrUsedSize(size_);

    // Remove the file from storage
    localStorage->getContent()->erase(fullpath_);

    return 0;
  }
}

}} // namespace simgrid::s4u
