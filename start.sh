
gnome-terminal --tab -- bash -c "roscore; exec bash"
sleep 2
source /home/nvidia/jihousai/chajian/devel/setup.bash 
gnome-terminal --tab -- bash -c "python3 /home/wanji/codebag/onsite-structured-test-master/onsite-structured-test/main.py; exec bash"

