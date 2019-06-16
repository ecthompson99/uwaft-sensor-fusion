sudo add-apt-repository ppa:git-core/ppa
sudo apt-get update

sudo apt-get install git
git config core.hooksPath .githooks
sudo apt install clang-format

chmod +x .githooks/apply-format
chmod +x .githooks/pre-commit
