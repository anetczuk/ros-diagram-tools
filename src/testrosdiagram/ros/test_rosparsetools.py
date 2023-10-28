# Copyright (c) 2022, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#

import unittest

from rosdiagram.ros.rosparsetools import parse_node_info_file, extract_topics_from_node_data, \
    extract_services_from_node_data

from testrosdiagram.ros.data import get_data_path


class ROSParseTest(unittest.TestCase):

    def setUp(self):
        ## Called before testfunction is executed
        pass

    def tearDown(self):
        ## Called after testfunction was executed
        pass

    def test_parse_node_info_file(self):
        node_path = get_data_path( "_gazebo.txt" )
        node_data = parse_node_info_file( node_path )

        topics_list = extract_topics_from_node_data( node_data )
        self.assertEqual( len(topics_list), 11 )
        self.assertTrue( "/rosout" in topics_list )

        services_list = extract_services_from_node_data( node_data )
        self.assertEqual( len(services_list), 30 )
        self.assertTrue( "/gazebo/get_loggers" in services_list )
