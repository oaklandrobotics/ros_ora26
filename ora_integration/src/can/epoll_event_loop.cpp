#include "../../include/can/epoll_event_loop.hpp"

EpollEventLoop::EpollEventLoop()
{
  // Open an epoll file descriptor (epollfd) to monitor for CAN events
  epollfd = epoll_create1(0);

  // Create eventfd used for shutdown
  shutdown_fd_ = eventfd(0,0);

  // Register that event to call during shutdown
  // Pass a blank callback
  register_event(
    &shutdown_ctx, shutdown_fd_, EPOLLIN,
    [this](uint32_t event_mask)
    {
      on_shutdown_event(event_mask);
    }
  );
}

EpollEventLoop::~EpollEventLoop()
{
  deregister_event(shutdown_ctx);
  close(shutdown_fd_);
  close(epollfd);
}

bool EpollEventLoop::register_event(EvtId* p_evt, int fd, uint32_t events, const Callback& callback)
{
  // Associate a callback to each file descriptor
  EventContext* ctx = new EventContext{ fd, callback };

  struct epoll_event ev = {
    .events = events,
    .data = { .ptr = ctx }
  };

  // Add the epoll events to the epollfd
  if (epoll_ctl(epollfd, EPOLL_CTL_ADD, fd, &ev) == -1)
  {
    delete ctx; // Cleanup the dynamically allocated EventContext in case of failure
    return false;
  }

  if (p_evt) *p_evt = ctx;

  n_events_++;
  return true;
}

bool EpollEventLoop::deregister_event(EvtId evt)
{
  if (evt == nullptr)
  {
    return false;
  }
  if (epoll_ctl(epollfd, EPOLL_CTL_DEL, evt->fd, nullptr) == -1)
  {
    return false;
  }

  drop_event(evt);
  delete evt;

  n_events_--;
  return true;
}

bool EpollEventLoop::run()
{
  if (stop_)
  {
    stop_ = false;
  }

  // Continue until there are no registered fds (Continue while CAN socket is registered)
  while (n_events_) {
    // Wait indefinitely for events to be triggered
    n_triggered_events_ = epoll_wait(epollfd, triggered_events_, kMaxEventsPerIteration, -1);
    
    if (n_triggered_events_ == -1)
    {
      return false;
    }

    // Process each event returned by epoll_wait
    for (int i = 0; i < n_triggered_events_; ++i)
    {
      EventContext* handler = static_cast<EventContext*>(triggered_events_[i].data.ptr);

      // Execute the callback associated with the fd
      handler->callback(triggered_events_[i].events);
    }

    if (stop_)
    {
      return true;
    }
  }

  return true;
}

void EpollEventLoop::drop_event(EvtId evt)
{
  for (int i = 0; i < n_triggered_events_; ++i)
  {
    if (reinterpret_cast<EventContext*>(triggered_events_[i].data.ptr) == evt)
    {
      triggered_events_[i].data.ptr = nullptr;
    }
  }
}

void EpollEventLoop::stop_loop()
{
  stop_ = true;

  const uint64_t val = 1;

  if (write(shutdown_fd_, &val, sizeof(val)) != sizeof(val))
  {
    throw std::runtime_error("Error occured while writing value to eventfd to break epoll_wait!");
  }
}

bool EpollEventLoop::is_stopped()
{
  return stop_.load();
}

void EpollEventLoop::on_shutdown_event(uint32_t event_mask)
{
  // Consume the counter by reading the shutdown_fd_
  uint64_t val;
  read(shutdown_fd_, &val, sizeof(val));
}

bool EpollEvent::init(EpollEventLoop* event_loop, const Callback& callback)
{
  event_loop_ = event_loop;
  callback_ = callback;

  fd_ = eventfd(0, 0);
  if (fd_ < 0)
  {
    return false;
  }

  if (!event_loop->register_event(&evt_, fd_, EPOLLIN, std::bind(&EpollEvent::on_trigger, this, _1)))
  {
    close(fd_);
    std::cerr << "Failed to register event" << std::endl;
    return false;
  }

  return true;
}

void EpollEvent::deinit()
{
  event_loop_->deregister_event(evt_);
  close(fd_);
  fd_ = -1;
}

bool EpollEvent::set()
{
  const uint64_t val = 1;

  if (write(fd_, &val, sizeof(val)) != sizeof(val))
  {
    return false;
  }

  return true;
}

void EpollEvent::on_trigger(uint32_t event_id)
{
  uint64_t val;
  if (read(fd_, &val, sizeof(val)) != sizeof(val))
  {
    std::cerr << "Failed to read eventfd" << std::endl;
    return;
  }

  callback_(event_id);
}