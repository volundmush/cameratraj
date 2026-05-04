#!/usr/bin/env bash

set -euo pipefail

usage() {
  printf 'Usage: %s [animation|stills] [frames] [width] [height]\n' "$0"
}

job_type="${1:-animation}"
frames="${2:-120}"
width="${3:-640}"
height="${4:-360}"

if [[ "$job_type" == "stills" ]]; then
  modes=(basic reflection refraction glossy softshadow all)
  views=(front left right top)
  for mode in "${modes[@]}"; do
    for view in "${views[@]}"; do
      python3 rayTrace.py --mode "$mode" --view "$view" --width "$width" --height "$height" &
    done
  done
  wait
  exit 0
fi

if [[ "$job_type" != "animation" ]]; then
  usage
  exit 2
fi

mkdir -p frames outputs
python3 cameraTrajectory.py --visualize --frames "$frames"

for ((frame = 0; frame < frames; frame++)); do
  python3 cameraTrajectory.py \
    --method se3 \
    --mode all \
    --frames "$frames" \
    --frame "$frame" \
    --width "$width" \
    --height "$height" \
    --output frames &
done

wait

if command -v ffmpeg >/dev/null 2>&1; then
  ffmpeg -y -framerate 24 -i frames/se3_%04d.png -pix_fmt yuv420p outputs/se3_camera_trajectory.mp4
else
  printf 'ffmpeg not found; rendered PNG frames are in frames/.\n'
fi
