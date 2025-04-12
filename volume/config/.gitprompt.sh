##########################################################################
#
# file: .gitprompt.sh
# date:  Sáb Dez 29 00:00:01 -02 2018
# author: Iuro Nascimento
# description: adapted bash prompt to colorize in gruvbox and to add git
# colors of having staged files, unstaged files, mixed and a clean HEAD
#
##########################################################################

# Prompt functions and definition
# GREEN_GITS='\[\033[01;32m\]'
GREEN_GITS=$(tput setaf 106)
# GREEN_PMT='\[\033[01;32m\]'
# WHITE_PMT='\[\033[00m\]'
# BLUE_PMT='\[\033[01;34m\]'
# RED_PMT='\[\033[01;31m\]'
# GREEN_EPMT='\033[01;32m'
# WHITE_EPMT='\033[00m'
# BLUE_EPMT='\033[01;34m'
# RED_EPMT='\033[01;31m'

function nonzero_return() {
	RETVAL=$?
	[ $RETVAL -ne 0 ] && echo "$RETVAL"
}

############# I couldnt find the base code for these git parse functions
# but I did altered then from a gist somewere
# get current branch in git repo
function parse_git_branch() {
	# BRANCH=`git branch 2> /dev/null | sed -e '/^[^*]/d' -e 's/* \(.*\)/\1/'`
	BRANCH=`git branch 2> /dev/null | sed -n 's/* \(.*\)/\1/p'`
	if [ ! "${BRANCH}" == "" ]
	then
		# STAT=`parse_git_dirty`
		echo "${BRANCH}"
	else
		echo ""
	fi
}

function parse_git_status() {
	LG=`locale | grep LANG=pt_BR`
	if [ -z "$LG" ]
	then
		dirty_s="modified:"
		untracked_s="Untracked files"
		ahead_s="Your branch is ahead of"
		newfile_s="new file:"
		renamed_s="renamed:"
		deleted_s="deleted:"
		unstaged_s="Changes not staged for commit:"
		staged_s="Changes to be committed:"
		clean_s="nothing to commit, working tree clean"
	else
		dirty_s="modificado:"
		untracked_s="Ficheiros desmonitorizados:"
		ahead_s="Seu ramo está à frente de"
		newfile_s="Ficheiros desmonitorizados:"
		renamed_s="nome mudado:"
		deleted_s="eliminado:"
		unstaged_s="Alterações por encenar para memória:"
		staged_s="Alterações para serem memorizadas:"
		clean_s="nada a memorizar, árvore-trabalho limpa"
	fi


	status=`git status 2>&1 | tee`
	dirty=`echo -n "${status}" 2> /dev/null | grep "${dirty_s}" &> /dev/null; echo "$?"`
	untracked=`echo -n "${status}" 2> /dev/null | grep "${untracked_s}" &> /dev/null; echo "$?"`
	ahead=`echo -n "${status}" 3> /dev/null | grep "${ahead_s}" &> /dev/null; echo "$?"`
	newfile=`echo -n "${status}" 2> /dev/null | grep "${newfile_s}" &> /dev/null; echo "$?"`
	renamed=`echo -n "${status}" 2> /dev/null | grep "${renamed_s}" &> /dev/null; echo "$?"`
	deleted=`echo -n "${status}" 2> /dev/null | grep "${deleted_s}" &> /dev/null; echo "$?"`
	unstaged=`echo -n "${status}" 2> /dev/null | grep "${unstaged_s}" &> /dev/null; echo "$?"`
	staged=`echo -n "${status}" 2> /dev/null | grep "${staged_s}" &> /dev/null; echo "$?"`
	clean=`echo -n "${status}" 2> /dev/null | grep "${clean_s}" &> /dev/null; echo "$?"`
	bits=''
	if [ "${dirty}" == "0" ]; then
		bits="${bits}1"
	else
		bits="${bits}0"
	fi
	if [ "${untracked}" == "0" ]; then
		bits="${bits}1"
	else
		bits="${bits}0"
	fi
	if [ "${ahead}" == "0" ]; then
		bits="${bits}1"
	else
		bits="${bits}0"
	fi
	if [ "${newfile}" == "0" ]; then
		bits="${bits}1"
	else
		bits="${bits}0"
	fi
	if [ "${renamed}" == "0" ]; then
		bits="${bits}1"
	else
		bits="${bits}0"
	fi
	if [ "${deleted}" == "0" ]; then
		bits="${bits}1"
	else
		bits="${bits}0"
	fi
	if [ "${unstaged}" == "0" ]; then
		bits="${bits}1"
	else
		if [ "${untracked}" == "0" ]; then
			bits="${bits}1"
		else
			bits="${bits}0"
		fi
	fi
	if [ "${staged}" == "0" ]; then
		bits="${bits}1"
	else
		bits="${bits}0"
	fi
	if [ "${clean}" == "0" ]; then
		bits="${bits}1"
	else
		bits="${bits}0"
	fi
	echo $bits
}

function print_git_status() {
	BITS=$(parse_git_status)
	dirty=${BITS:0:1}
	untracked=${BITS:1:1}
	ahead=${BITS:2:1}
	newfile=${BITS:3:1}
	renamed=${BITS:4:1}
	deleted=${BITS:5:1}
	unstaged=${BITS:6:1}
	staged=${BITS:7:1}
	clean=${BITS:8:1}
	if [ "${dirty}" == "1" ]; then
		echo "modified"
	fi
	if [ "${untracked}" == "1" ]; then
		echo "untracked"
	fi
	if [ "${ahead}" == "1" ]; then
		echo "ahead"
	fi
	if [ "${newfile}" == "1" ]; then
		echo "newfile"
	fi
	if [ "${renamed}" == "1" ]; then
		echo "renamed"
	fi
	if [ "${deleted}" == "1" ]; then
		echo "deleted"
	fi
	if [ "${unstaged}" == "1" ]; then
		echo "unstaged"
	fi
	if [ "${staged}" == "1" ]; then
		echo "staged"
	fi
	if [ "${clean}" == "1" ]; then
		echo "clean"
	fi
}

# source ~/bin/git-prompt.sh

function test_status() {
	RETVAL=$?
	if [ $RETVAL -ne 0 ]; then
		echo "carai"
	else
		echo "ola"
	fi

}
function reset_ps1() {
	local RETVAL=$?

# color schemes from @0xADADA’s bash prompt
# https://github.com/0xadada/dotfiles/blob/master/.bash_prompt
	# if [[ $COLORTERM = gnome-* && $TERM = xterm ]] && infocmp gnome-256color >/dev/null 2>&1; then
	# 	export TERM=gnome-256color
	# elif infocmp rxvt-unicode-256color >/dev/null 2>&1; then
	# 	export TERM=rxvt-unicode-256color
	# elif infocmp xterm-256color >/dev/null 2>&1; then
	# 	export TERM=xterm-256color
	# fi
	BOLD=""
	RESET=""
	BLACK=""
	BLACK1=""
	BLACK1_BG=""
	BLACK2=""
	BLACK2_BG=""
	RED=""
	RED_BG=""
	GREEN=""
	YELLOW=""
	BLUE=""
	PURPLE=""
	AQUA=""
	GRAY=""
	WHITE=""
	ORANGE=""
	ORANGE_BG=""

	if tput setaf 1 &> /dev/null; then
		tput sgr0
		BOLD=$(tput bold)
		RESET=$(tput sgr0)
		if [[ $(tput colors) -ge 256 ]] 2>/dev/null; then
		    BLACK=$(tput setaf 235)
		    BLACK1=$(tput setaf 237)
		    BLACK1_BG=$(tput setab 237)
		    BLACK2=$(tput setaf 239)
		    BLACK2_BG=$(tput setab 239)
		    RED=$(tput setaf 124)
		    RED_BG=$(tput setab 124)
		    GREEN=$(tput setaf 142)
		    YELLOW=$(tput setaf 214)
		    BLUE=$(tput setaf 66)
		    PURPLE=$(tput setaf 175)
		    AQUA=$(tput setaf 108)
		    GRAY=$(tput setaf 246)
		    GRAY_BG=$(tput setab 246)
		    WHITE=$(tput setaf 223)
		    ORANGE=$(tput setaf 208)
		    ORANGE_BG=$(tput setab 208)
		else
		    BLACK=$(tput setaf 0)
		    BLACK1=$(tput setaf 0)
		    BLACK1_BG=$(tput setab 0)
		    BLACK2=$(tput setaf 0)
		    BLACK2_BG=$(tput setab 0)
		    RED=$(tput setaf 1)
		    RED_BG=$(tput setab 1)
		    GREEN=$(tput setaf 2)
		    YELLOW=$(tput setaf 3)
		    BLUE=$(tput setaf 4)
		    PURPLE=$(tput setaf 13)
		    AQUA=$(tput setaf 6)
		    GRAY=$(tput setaf 7)
		    GRAY_BG=$(tput setab 7)
		    WHITE=$(tput setaf 7)
		    ORANGE=$(tput setaf 3)
		fi
	fi
	WHITE_PMT=$WHITE
	RED_PMT=$RED
	GREEN_PMT=$GREEN
	# GREEN_GITS=$GREEN
	BLUE_PMT=$BLUE

	PS1="\001$BOLD\002"
	# if [ $RETVAL -ne 0 ]; then
	# 	PS1+="\001$RED_PMT\002➜"
	# else
	# 	PS1+="\001$BLUE_PMT\002➜"
	# fi

	# GIT_BRANCH=$(__git_ps1 "%s")
	GIT_BRANCH=$(parse_git_branch)
	GIT_STATUS=$(parse_git_status)
	PS1+="\001$GREEN_PMT\002\u@\h \001$BLUE_PMT\002\W\001$RESET\002"
	unstaged=${GIT_STATUS:6:1}
	staged=${GIT_STATUS:7:1}
	clean=${GIT_STATUS:8:1}

	glyph_color=''
	branch_color=''
	if [ ! "${GIT_BRANCH}" == "" ]; then
		if [ "${clean}" == "1" ]; then
			glyph_color="$WHITE_PMT"
			branch_color="$WHITE_PMT"
		fi
		if [ "${unstaged}" == "1" ]; then
			if [ "${staged}" == "1" ]; then
				glyph_color="$GREEN_GITS"
			else
				glyph_color="$RED_PMT"
			fi
			branch_color="$RED_PMT"
		else
			if [ "${staged}" == "1" ]; then
				glyph_color="$GREEN_GITS"
				branch_color="$GREEN_GITS"
			fi
		fi
		PS1+=" \001$BOLD\002\001$BLUE_PMT\002[\001$glyph_color\002 \001$branch_color\002$GIT_BRANCH\001$BOLD\002\001$BLUE_PMT\002]"
	fi
	PS1+=" \001$BOLD\002\001$GREEN_PMT\002∫ "

	if [ $RETVAL -ne 0 ]; then
		PS1+="\001$RED_PMT\002✗ "
	fi
	PS1+="\001$WHITE_PMT\002\001$RESET\002"
}

export PROMPT_COMMAND='reset_ps1; history -a;'$PROMPT_COMMAND

