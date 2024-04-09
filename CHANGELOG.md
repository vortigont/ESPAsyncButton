# Change Log

## release v1.1.0
_new features:_
  Implement AsyncEventButton class

_bugfixes:_
 - fix bug when autorepeat was engaged when longPress timer expired during gpio debounce interval
 - fix gpio interrupt handling bug when debouncing was not working properly
 - set GenericButton::Counters members type to int32_t

## rel v1.0.0
### reworking code of rwmingis/InterruptButton lib
 - implement GenericButton class, event policy, event loop hooks, callback menu, gpio button