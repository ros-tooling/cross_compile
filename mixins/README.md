Check your mixin linting by running:
```bash
python3 lint.py
```
Your mixin arguments need to be sorted alphabetically, otherwise you
will get linting errors.

Install these mixins using
```bash
colcon mixin add cc_mixins file://<path_to_cross_compile_repo>/mixins/index.yaml
colcon mixin update cc_mixins
```
where `cc_mixins` will be the name of the mixin group (which can be changed).

Check the mixins are installed by running
```bash
colcon mixin show
```
