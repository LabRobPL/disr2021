
#pragma once

// IGN_DEPRECATED is defined by all ignition libraries, but the version below
// is a simplified version.  When mixing the regular ignition libraries and
// the drake compiled ignition libraries, the compiler throws a warning about
// the macro being multiply defined.  We undefine it before redefining it here
// to work around that issue.  Note that the IGNITION_MATH_VISIBLE macro
// shouldn't be defined multiple times, but we undefine it just in case.

#ifdef IGNITION_MATH_VISIBLE
#undef IGNITION_MATH_VISIBLE
#endif
#define IGNITION_MATH_VISIBLE __attribute__ ((visibility("default")))

#ifdef IGN_DEPRECATED
#undef IGN_DEPRECATED
#endif
#define IGN_DEPRECATED(version) __attribute__ ((__deprecated__))
    