#!/usr/bin/env python3
#
# MIT License
#
# Copyright (c) 2022 Arkadiusz Netczuk <dev.arnet@gmail.com>
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#

try:
    ## following import success only when file is directly executed from command line
    ## otherwise will throw exception when executing as parameter for "python -m"
    # pylint: disable=W0611
    import __init__
except ImportError:
    ## when import fails then it means that the script was executed indirectly
    ## in this case __init__ is already loaded
    pass


# import sys
import os

import logging
import unittest
import re
import argparse


script_dir = os.path.dirname(os.path.abspath(__file__))

# src_dir = os.path.abspath(os.path.join(script_dir, ".."))
# sys.path.insert(0, src_dir)


_LOGGER = logging.getLogger(__name__)


def match_tests( pattern: str ):
    if pattern.find("*") < 0:
        ## regular module
        loader = unittest.TestLoader()
        return loader.loadTestsFromName( pattern )

    ## wildcarded
    rePattern = pattern
    # pylint: disable=W1401
    rePattern = rePattern.replace( "/", "." )
    rePattern = rePattern.replace( ".", r"\." )
    rePattern = rePattern.replace( "*", ".*" )
    ## rePattern = "^" + rePattern + "$"
    _LOGGER.info( "searching test cases with pattern: %s", rePattern )
    loader = unittest.TestLoader()
    testsSuite = loader.discover( script_dir )
    return match_test_suites(testsSuite, rePattern)


def match_test_suites( testsList, rePattern: str ):
    retSuite = unittest.TestSuite()
    for testObject in testsList:
        if isinstance(testObject, unittest.TestSuite):
            subTests = match_test_suites( testObject, rePattern )
            retSuite.addTest( subTests )
            continue
        if isinstance(testObject, unittest.TestCase):
            classobj         = testObject.__class__
            # pylint: disable=W0212,
            testCaseFullName = ".".join([ classobj.__module__, classobj.__name__,
                                          testObject._testMethodName ] )
            matched = re.search(rePattern, testCaseFullName)
            if matched is not None:
                ## _LOGGER.info("test case matched: %s", testCaseFullName )
                retSuite.addTest( testObject )
            continue
        _LOGGER.warning("unknown type: %s", type( testObject ))
    return retSuite


## ============================= main section ===================================


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Test runner')
    parser.add_argument('-la', '--logall', action='store_true', help='Log all messages' )
    # pylint: disable=C0301
    parser.add_argument('-rt', '--run_test', action='store', required=False, default="",
                        help='Module with tests, e.g. module.submodule.test_file.test_class.test_method, wildcard * allowed' )
    parser.add_argument('-r', '--repeat', action='store', type=int, default=0, help='Repeat tests given number of times' )
    parser.add_argument('-ut', '--untilfailure', action="store_true", help='Run tests in loop until failure' )
    parser.add_argument('-v', '--verbose', action="store_true", help='Verbose output' )

    args = parser.parse_args()

    logging.basicConfig()
    if args.logall is True:
        logging.getLogger().setLevel( logging.DEBUG )
    else:
        logging.getLogger().setLevel( logging.ERROR )

    verbosity = 1
    if args.verbose:
        verbosity = 2

    if args.run_test:
        ## not empty
        suite = match_tests( args.run_test )
    else:
        testsLoader = unittest.TestLoader()
        suite = testsLoader.discover( script_dir )

    testsRepeats = int(args.repeat)

    ## run proper tests
    if args.untilfailure is True:
        counter = 1
        while True:
            print( "Tests iteration:", counter )
            counter += 1
            testResult = unittest.TextTestRunner(verbosity=verbosity).run(suite)
            if testResult.wasSuccessful() is False:
                break
            print( "\n" )
    elif testsRepeats > 0:
        for counter in range(1, testsRepeats + 1):
            print( "Tests iteration:", counter )
            testResult = unittest.TextTestRunner(verbosity=verbosity).run(suite)
            if testResult.wasSuccessful() is False:
                break
            print( "\n" )
    else:
        unittest.TextTestRunner(verbosity=verbosity).run(suite)
