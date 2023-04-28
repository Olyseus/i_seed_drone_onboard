#!/bin/bash

set -e

temp_file=/tmp/__clang_format_check.txt

echo "Temp: ${temp_file}"
echo "Clang format: `which clang-format`"

for to_check in `find . -type f \( -name '*.cpp' -or -name '*.h' -or -name '*.hpp' \) ! -path './_builds/*' ! -path './third_party/*' ! -path "${HOME}/opt" ! -path './_deps/*' ! -path './dji_linux_*' ! -path './osdk*_linux*' ! -path './common/osal/osal*.h' ! -path './manifold2/hal/hal*.h' ! -path './laser/application.*pp' ! -path './laser/dji_sdk_app_info.h'`;
do
  echo "Checking ${to_check} ..."
  echo "To diff: ${to_check} ${temp_file}"
  clang-format -style=file ${to_check} > ${temp_file}
  diff ${to_check} ${temp_file}
done
