DELAYED_SYNC module:
Module implements synchronization logic for delayed read or write requests.
Flags are synchronized to pass from requesting clock domain to completing clock domain
and back.
It also implements completion expiration counter and max_retry expired request ditching to 
avoid deadlocks.