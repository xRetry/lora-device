apt-get install -y curl unzip g++
mkdir -p $HOME/.config

# NVIM
curl -LO https://github.com/neovim/neovim/releases/download/stable/nvim-linux64.tar.gz
tar xzvf nvim-linux64.tar.gz
mkdir -p $HOME/.local/share/nvim-linux64
mv nvim-linux64 $HOME/.local/share/
ln -s $HOME/.local/share/nvim-linux64/bin/nvim /usr/local/bin/nvim
rm nvim-linux64.tar.gz
# Config
mkdir -p $HOME/.config/nvim
git clone https://github.com/xRetry/nvim.git $HOME/.config/nvim
nvim --headless +'Lazy sync' +'sleep 20' +qall

# TMUX
apt-get install -y tmux
# Config
mkdir -p $HOME/.config/tmux
git clone https://github.com/xRetry/tmux.git $HOME/.config/tmux
echo "alias tmux='TERM=xterm-256color tmux -f $HOME/.config/tmux/tmux.conf'" >> $HOME/.bashrc

echo "export TERM=screen-256color-bce" >> $HOME/.bashrc

nvim --headless +'TSInstall c cpp' +'MasonInstall clangd' +'sleep 40' +qall

