# Copyright (c) 2022, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#

def code_inline( data ):
    return f"`{data}`"


def link_simple( label, url ):
    return f"[{label}]({url})"
