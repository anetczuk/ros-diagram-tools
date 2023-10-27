# Copyright (c) 2022, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#


def code_inline( text ):
    return f"<code>{text}</code>"


def link_simple( label, url ):
    return f"""<a href="{ convert_autolink(url) }">{label}</a>"""


def convert_autolink( url ):
    if not url.endswith(".autolink"):
        return url
    return f"{url[:-8]}html"
