alias fzt="fzf-tmux"
alias zb="z -b"		 # search back in the pwd
alias zh="z -I -t ." # history
alias zz="z -c"      # restrict matches to subdirs of $PWD
alias zi="z -i"      # cd with interactive selection
alias zf="z -I"      # use fzf to select in multiple matches

# some more ls aliases
alias ls="ls --color"
alias ll="ls -alF --color"
alias la="ls -A --color"
alias l="ls -CF"

# apt aliases
alias apti="sudo apt-get install -y"
alias aptu="sudo apt-get update"
alias aptdu="sudo apt-get dist-upgrade"
alias aptug="sudo apt-get upgrade"
alias apta="sudo apt-get autoremove"
alias add-repo="sudo add-apt-repository"
alias apts="apt-cache search"
alias aptp="sudo apt-get purge"
alias smi="sudo make install"
alias mi="make install"

# pacman 
alias pac="sudo pacman -S"
alias pacu="sudo pacman -Syu"
alias pacs="pacman -Ss"
alias pacr="sudo pacman -R"

alias em="emacs -nw"
alias helgrind="valgrind --tool=helgrind"
alias cgr="find . -name '*.h' -or -name '*.c' -or -name '*.cpp' -or -name '*.cc' | xargs grep \!* /dev/null"
alias check="/home/gagarin/git/kernel/linux/scripts/checkpatch.pl"
alias cm="catkin_make --cmake-args --parallel 4"
alias cmr="rm -rf ~/catkin_make/build/* ~/catkin_ws/devel/* && catkin_make --cmake-args --parallel 4"

alias gs="git status "
alias ga="git add "
alias gb="git branch "
alias gc="git commit"
alias gd="git diff"
# alias go="git checkout "
alias got="git checkout"
alias gr="git remote"
alias gk="gitk --all&"
alias gx="gitx --all"
alias gu="git unstage"
alias gln="git log --oneline -n"
alias gl="git log --oneline"
alias gsl="git stash list"

alias cls="clear"

alias tmux="tmux -2"
alias tm="tmux"
alias tma="tmux attach"
alias tmk="tmux kill-session"
alias tmks="tmux kill-server"
alias vtm="nvim ~/.config/tmux/tmux.conf"

alias pdflatex="pdflatex -shell-escape"
alias mpad="mousepad"
# alias vim="vim --servername vim"

alias v="nvim"
alias sv="sudo nvim"
alias vrc="vim ~/.vimrc"
alias vbrc="vim ~/.bashrc"
alias brc="~/.bashrc"
alias zrc="~/.zshrc"
alias bal="~/.bash_aliases"
alias vzrc="vim ~/.zshrc"
alias frc="~/.config/fish/config.fish"
alias fal="~/.config/fish/.fish_aliases"
alias vish="vim ~/.config/fish/config.fish"
alias val="vim ~/.config/fish/.fish_aliases"
alias sros="source /opt/ros/jazzy/setup.bash"
alias sws="source ~/ros2_ws/install/setup.bash"
# unalias fd
alias zat="zathura"
alias term_size="echo 'Rows=$(tput lines) Cols=$(tput cols)'"
alias free="free -h"
alias zt="zathura"
# alias git="hub"
alias vw="vim -c VimwikiIndex"
alias jn="jupyter notebook"
# source ~/.aliasme/aliasme.sh
# ENACOM aliases
alias fw="python flask_wrapper.py"
alias we="workon enacom"
alias de="deactivate"
alias py="python3"
alias vf="fd . -tf --hidden | fzf --bind 'enter:become(nvim {})'"
alias ldo="lazydocker"
