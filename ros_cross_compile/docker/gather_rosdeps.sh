#!/bin/bash
set -euxo pipefail

if [ ! -d ./src ]; then
  echo "No src/ directory found at $(pwd), did you remember to mount your workspace?"
  exit 1
fi

if [ -f "${CUSTOM_SETUP}" ]; then
  chmod +x "${CUSTOM_SETUP}"
  pushd "$(dirname "${CUSTOM_SETUP}")"
  "${CUSTOM_SETUP}"
  popd
fi

out_dir=$(dirname "${OUT_PATH}")
mkdir -p "${out_dir}"

rosdep update

cat > "${OUT_PATH}" <<EOF
#!/bin/bash
set -euxo pipefail
EOF

mapfile -t package_paths < <(colcon list -p)

rosdep install \
    --os "${TARGET_OS}" \
    --rosdistro "${ROSDISTRO}" \
    --from-paths "${package_paths[@]}" \
    --ignore-src \
    --reinstall \
    --default-yes \
    --simulate \
  >> "${OUT_PATH}"

chmod +x "${OUT_PATH}"
chown -R "${OWNER_USER}" "${out_dir}"
