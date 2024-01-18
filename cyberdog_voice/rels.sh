colcon build

rm -r -f release
mkdir release

cp -r install release

cp startcybervoice.sh release
cp cybervoice.service release
cp installservice.sh release


