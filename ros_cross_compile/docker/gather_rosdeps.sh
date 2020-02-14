#!/bin/bash
set -euxo pipefail

[ "${ROSDISTRO}" ] || (echo "ROSDISTRO variable not set" && exit 1)
[ "${TARGET_OS}" ] || (echo "TARGET_OS variable not set" && exit 1)

if [ ! -d ./src ]; then
  echo "No src/ directory found at /root/ws, did you remember to mount your workspace?"
  exit 1
fi

internals_dir=cc_internals
mkdir -p "${internals_dir}"
rosdep_script="${internals_dir}"/install_rosdeps.sh

rosdep update

cat > "${rosdep_script}" <<EOF
#!/bin/bash
set -euxo pipefail
EOF

rosdep install \
    --os "${TARGET_OS}" \
    --rosdistro "${ROSDISTRO}" \
    --from-paths ./src  \
    --ignore-src \
    --reinstall \
    --default-yes \
    --simulate \
  >> "${rosdep_script}"

chmod +x "${rosdep_script}"
