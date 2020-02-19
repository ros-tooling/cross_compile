# Developing ros-cross-compile

This document is intended to contain instructions, guidelines, and requirements for developing this tool.

## Writing Code

It is recommended to use `virtualenv` to manage the dependency environment.

```
# create a virtualenv
virtualenv venv
# use it
source venv/bin/activate
# install this package in the virtualenv to get dependencies
pip install -e .
# now you can run your work
ros_cross_compile
```

## Tests

There are currently two test entrypoints

* Unit tests via `tox`
  * run `tox -e py`
* End-to-end shell script
  * run `./test/run_e2e_test.sh`

## Host Platforms

This tool aims to support as host platforms
* all ROS 2 Tier 1 platforms
* on x86_64 (cross-compiling _from_ ARM platforms has not been deemed necessary)
* for LTS distributions that have not hit EOL

See [REP 2000](https://www.ros.org/reps/rep-2000.html) to determine this target, which as of this writing is:
* Dashing Diademata
  * Ubuntu Bionic 18.04
    * support installation via `pip` and `apt`
  * MacOS Sierra (10.12)
    * support installation via `pip`
  * Windows 10 (VS2019)
    * support installation via `pip`

Though not all of these platforms may be supported yet, design decisions may not be made that rule out support for those platforms in the future.
