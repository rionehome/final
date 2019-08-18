#[C+ctrl]検知
trap 'last' {1,2,3,15}

last(){
	kill `ps aux | grep "ros" | grep -v "gnome-terminal" | awk '{print $2}'` &>/dev/null
	kill `ps aux | grep "ssh" | grep -v "gnome-terminal" | awk '{print $2}'` &>/dev/null
	exit 1
}

gnome-terminal --geometry=60x6 -x  bash -c  "
	roslaunch final final.launch
	sleep 10
" &

gnome-terminal --geometry=60x6 -x  bash -c  "
	ssh -R rione-final:80:localhost:3000 serveo.net
	sleep 10
" &

while :
do
	sleep 1
done

