# sunray_helper

`sunray_helper` is the user-facing facade for the Sunray UAV system.

Users are expected to include a single header:

```cpp
#include <sunray_helper/sunray_helper.h>
```

and then create a `sunray_helper::SunrayHelper` instance to access high-level
operations such as state access, takeoff, landing, and waypoint commands.

