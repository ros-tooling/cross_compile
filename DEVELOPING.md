# Developing ros-cross-compile

This document contains instructions, guidelines, and requirements for developing this tool.

## Writing Code

For easiest development, use `virtualenv` to manage the dependency environment.

```
# create a virtualenv
virtualenv venv
# use it
source venv/bin/activate
# install this package in the virtualenv to get dependencies
pip3 install -e .
# now you can run your work
ros_cross_compile
```

## Tests

These are testing entrypoints:

* Unit tests via `tox`
  * run `tox -e py`
* End-to-end shell script
  * run `./test/run_e2e_test.sh`

## Host Platforms

The target host platforms for this tool are the Tier 1 platforms specified by non-EOL ROS 2 LTS distributions, on x86_64.
Cross-compiling on ARM host platforms is out of scope.

See [REP 2000](https://www.ros.org/reps/rep-2000.html) for this list, which is now:
* Foxy Fitzroy
  * Ubuntu Focal 20.04
    * support installation via `pip3` and `apt`
  * MacOS Sierra (10.12)
    * support installation via `pip3`
  * Windows 10 (VS2019)
    * support installation via `pip3`

Though not all of these targets may be fully supported yet, design decisions may not be made that rule out support for those platforms in the future.

## Releasing ros-cross-compile

This just a simple reminder for the process to release:

1. `pip3 install -U twine setuptools`
1. Update version number in `setup.py`
1. Create a new git tag associated with that commit
  * Note, this needs to be the _actual commit_ that goes on `master` - so if you do a PR, wait until after the PR is merged to tag the new HEAD (in case of e.g. squash and merge)
1. `python3 setup.py sdist bdist_wheel`
1. Probably try testpypi first: `twine upload -r testpypi dist/*` - then test it
1. Upload to pypi: `twine upload dist/*`
