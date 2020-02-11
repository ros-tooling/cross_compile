#!/bin/bash
set -euxo pipefail

[ "${ROSDISTRO}" ] || (echo "ROSDISTRO var unset" && exit 1)

ls

if [ ! -d ./src ]; then
  echo "No src directory found at /root/ws, did you remember to mount your workspace?"
  exit 1
fi

internals_dir=cc_internals
mkdir -p $internals_dir
rosdep_script=$internals_dir/install_rosdeps.sh

rosdep update

echo "#!/bin/bash" > $rosdep_script
echo "set -euxo pipefail" >> $rosdep_script
rosdep install \
    --from-paths ./src  \
    --ignore-src \
    --simulate \
    -y \
    --rosdistro "${ROSDISTRO}" \
  >> $rosdep_script

chmod +x $rosdep_script
