export PYENV_ROOT="$HOME/.pyenv"
command -v pyenv >/dev/null || export PATH="$PYENV_ROOT/bin:$PATH"
eval "$(pyenv init -)"

eval "$(pyenv virtualenv-init -)"

# >>> conda initialize >>>
# !! Contents within this block are managed by 'conda init' !!
__conda_setup="$('/home/hrsa/miniconda3/bin/conda' 'shell.bash' 'hook' 2> /dev/null)"
if [ $? -eq 0 ]; then
   eval "$__conda_setup"
else
   if [ -f "/home/hrsa/miniconda3/etc/profile.d/conda.sh" ]; then
       . "/home/hrsa/miniconda3/etc/profile.d/conda.sh"
   else
       export PATH="/home/hrsa/miniconda3/bin:$PATH"
   fi
fi

unset __conda_setup
# <<< conda initialize <<<

# Pythonpath has to be set for the conda environment by using:
PYTHONPATH=/home/hrsa/miniconda3/bin/python

# activate conda environment via ~$ 
source /home/hrsa/miniconda3/bin/activate
