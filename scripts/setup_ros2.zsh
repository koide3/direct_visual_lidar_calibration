# Source this file from zsh; it configures Humble and this workspace's local
# dependency prefix. It does not edit shell startup files or system libraries.
typeset _dual_mei_source_dir="${${(%):-%x}:A:h}"
typeset _dual_mei_workspace="${_dual_mei_source_dir:h:h:h}"
source /opt/ros/humble/setup.zsh || return $?
if [[ -d "${_dual_mei_workspace}/deps/system-root/usr" ]]; then
  export CMAKE_PREFIX_PATH="${_dual_mei_workspace}/deps/system-root/usr${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
  export LD_LIBRARY_PATH="${_dual_mei_workspace}/deps/system-root/usr/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
fi
if [[ -d "${_dual_mei_workspace}/deps/install" ]]; then
  export CMAKE_PREFIX_PATH="${_dual_mei_workspace}/deps/install${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
  export LD_LIBRARY_PATH="${_dual_mei_workspace}/deps/install/lib:${_dual_mei_workspace}/deps/install/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
fi
if [[ -f "${_dual_mei_workspace}/install/setup.zsh" ]]; then
  source "${_dual_mei_workspace}/install/setup.zsh" || return $?
fi
unset _dual_mei_source_dir _dual_mei_workspace
