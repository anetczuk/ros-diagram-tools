#!/usr/bin/env python3
#
# Copyright (c) 2023, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#

#
# Script checks for invalid links (a href) or invalid paths (img src) in Markdown files.
# Script converts md to html and then iterates through links and paths.
#
# Dependencies:
#    pip3 install --user markdown validators
#

import os
import sys
import logging
import argparse
import re

from glob import glob
import markdown
import validators

from bs4 import BeautifulSoup


_LOGGER = logging.getLogger(__name__)

SCRIPT_DIR = os.path.dirname( os.path.abspath(__file__) )


## ===================================================================


class FileChecker:

    def __init__(self, md_path):
        self.md_file = md_path
        md_dir = os.path.dirname(md_path)
        self.md_dir = os.path.abspath(md_dir)
        self.soup = None
        self.local_targets = None

        self._load()

    def _load(self):
        with open(self.md_file, 'r', encoding="utf-8") as file:
            file_content = file.read()
        html_content = markdown.markdown( file_content )
        with open("/tmp/page.txt", 'w', encoding="utf-8") as file:
            file.write(html_content)
        self.soup = BeautifulSoup(html_content, 'html.parser')
        self.local_targets = get_targets(self.soup)

    # return 'True' is everything ok, otherwise 'False'
    def checkMDFile(self):
        valid = True
        valid = self.checkA() and valid
        valid = self.checkImg() and valid
        return valid

    def checkA(self):
        valid = True
        for link in self.soup.find_all('a'):
            # link_name = link.get('name')
            link_href = link.get('href')
            if self._checkHref(link_href):
                continue
            _LOGGER.warning( "invalid link: %s in %s", link_href, self.md_file )
            valid = False
        return valid

    def checkImg(self):
        valid = True
        for img in self.soup.find_all('img'):
            img_src = img.get('src')
            if self._checkFile(img_src):
                continue
            _LOGGER.warning( "invalid path: %s in %s", img_src, self.md_file )
            valid = False
        return valid

    def checkLocalTarget(self, target_label):
        if target_label in self.local_targets:
            # found local target
            return True
        # invalid target
        return False

    def _checkHref(self, link_href):
        if link_href is None:
            # no href - name case
            return True
        if link_href.startswith("mailto:"):
            return True
        if self._checkFile(link_href):
            # valid file
            return True
        local_path = os.path.join( self.md_dir, link_href )
        if os.path.isdir(local_path):
            # valid directory
            return True

        target_data = link_href.split("#")
        if len(target_data) != 2:
            # invalid URL
            return False

        # url with target
        target_url = target_data[0]
        target_label = target_data[1]
        if not target_url:
            # local file
            if self.checkLocalTarget(target_label):
                # found local target
                return True

        # external file
        local_path = os.path.join( self.md_dir, target_url )
        if os.path.isdir(local_path):
            local_path = os.path.join( local_path, "README.md" )
        if not os.path.isfile(local_path):
            # invalid file
            return False
        checker = FileChecker(local_path)
        return checker.checkLocalTarget(target_label)

    def _checkFile(self, file_href):
        if validators.url(file_href):
            # valid link
            return True
        local_path = os.path.join( self.md_dir, file_href )
        if os.path.isfile(local_path):
            # valid file
            return True
        # invalid
        return False


def get_targets(soup):
    # on GitHub headers are converted to targets
    header_labels = extract_header_labels(soup)

    anchor_targets = set()
    for link in soup.find_all('a'):
        link_id = link.get('id')
        if link_id:
            anchor_targets.add(link_id)
        link_name = link.get('name')
        if link_name:
            anchor_targets.add(link_name)

    ret_data = set()
    ret_data.update( header_labels )
    ret_data.update( anchor_targets )
    return ret_data


def extract_header_labels(soup):
    header_items = set()
    header_items.update( soup.find_all('h1') )
    header_items.update( soup.find_all('h2') )
    header_items.update( soup.find_all('h3') )
    header_items.update( soup.find_all('h4') )
    header_items.update( soup.find_all('h5') )
    header_items.update( soup.find_all('h6') )
    return { item.text for item in header_items }


## ===================================================================


def find_md_files( search_dir ):
    ret_list = []
    for filename in glob(f'{search_dir}/**/*.md', recursive=True):
        ret_list.append(filename)
    return ret_list


def filter_items(items_list, ignore_patterns_list):
    _LOGGER.info( "ignore patterns: %s", ignore_patterns_list )
    ignored_list = []
    for item in items_list:
        for ignore_pattern in ignore_patterns_list:
            pattern = re.compile( ignore_pattern )
            item_match = pattern.match(item)
            if item_match:
                # matched
                ignored_list.append( item )
                break
    return ignored_list


def main():
    parser = argparse.ArgumentParser(description='dump tools')
    parser.add_argument( '-la', '--logall', action='store_true', help='Log all messages' )
    parser.add_argument( '-d', '--dir', action='store', help='Path to directory to search .md files and check' )
    parser.add_argument( '-i', '--ignore', action='store',
                         help='Semicolon separated list of regex expressions to ignore items' )
    parser.add_argument( '-f', '--file', action='store', help='Path to file to check' )

    args = parser.parse_args()

    logging.basicConfig()
    if args.logall is True:
        logging.getLogger().setLevel( logging.DEBUG )
    else:
        logging.getLogger().setLevel( logging.INFO )

    if not args.file and not args.dir:
        _LOGGER.error("argument required")
        return 1

    md_files = find_md_files(args.dir)
    if args.file:
        md_files.append(args.file)

    if args.ignore:
        ignore_patterns = args.ignore.split(";")
        ignored_list = filter_items(md_files, ignore_patterns)
        _LOGGER.info("ignored files:\n%s", '\n'.join(ignored_list))
        md_files = list( set(md_files) - set(ignored_list) )

    _LOGGER.info("files to check:\n%s", '\n'.join(md_files))

    valid = True
    for md_file in md_files:
        checker = FileChecker(md_file)
        if checker.checkMDFile() is False:
            valid = False

    if valid is False:
        # errors found
        _LOGGER.info("found invalid links")
        return 1

    # everything fine
    _LOGGER.info("links valid")
    return 0


if __name__ == '__main__':
    exit_code = main()
    sys.exit( exit_code )
