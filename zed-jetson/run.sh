docker run \
  --runtime=nvidia \
  --network=host \
  --privileged \
  -e DISPLAY="${DISPLAY}" \
  -e NVIDIA_DRIVER_CAPABILITIES="all" \
  -v /tmp:/tmp \
  -v /dev:/dev \
  -v /usr/local/zed/resources/:/usr/local/zed/resources/ \
  -v /usr/local/zed/settings/:/usr/local/zed/settings/ \
  dmocnik/ora_zed:latest