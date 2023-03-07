# Copyright (c) 2022, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#


def time_to_string( time_value, leading_zeros=True ):
    mins = int( time_value / 60 )
    secs = time_value % 60.0
    secs = round( secs, 2 )
    if not leading_zeros:
        return f"{mins} m {secs:0>4} s"
    return f"{mins:0>3} m {secs:0>4} s"
