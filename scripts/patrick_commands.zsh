#!/bin/bash

# Get the list of aliases
alias_commands=$(cat ~/catkin_ws/src/environmental_robot/scripts/patrick_alias.zsh | awk -F'[ =]' '{print $2}')

# Use fzf to select an alias
selected_alias=$(echo "$alias_commands" | fzf --height=40% --layout=reverse --info=inline --border)

# Run the selected alias
if [ -n "$selected_alias" ]; then
    # cat ~/catkin_ws/src/environmental_robot/scripts/patrick_alias.zsh | grep $selected_alias | cut -d'=' -f2-
    eval $(cat ~/catkin_ws/src/environmental_robot/scripts/patrick_alias.zsh | grep $selected_alias | cut -d'=' -f2-)
fi