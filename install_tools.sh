sudo add-apt-repository ppa:git-core/ppa
sudo apt-get update

sudo apt-get install git
git config core.hooksPath .githooks
sudo apt install clang-format

chmod +x .githooks/apply-format
chmod +x .githooks/pre-commit

toInstall=("ros-kinetic-desktop-full" "python-rosinstall" 
    "python-rosinstall-generator" "python-wstool" "build-essential" 
    "python-catkin-tools")

notInstalled=()
installed=()

softwareCount="0"

while [ $softwareCount -lt ${#toInstall[@]} ]
do
    if [ -n 'which' $toInstall(softwareCount)]; then
        installed(softwareCount)=${toInstall(softwareCount)}
    else
        notInstalled(softwareCount)=${toInstall(softwareCount)}
    fi
softwareCount=$[$softwareCount+1]
done

