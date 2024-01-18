colcon build

rm -r -f release
mkdir release

cp -r install release

cp startcybertail.sh release
cp cybertail.service release
cp startcybertail.sh release
cp installservice.sh release


