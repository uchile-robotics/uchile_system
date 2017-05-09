#!/usr/bin/env python
# -*- coding: utf-8 -*-

__author__ = 'Rodrigo Muñoz'
__email__ = 'rorro.mr@gmail.com'

import sys
import os
import signal
from threading import Thread, RLock
from time import sleep
import argparse

from gi.repository import Gtk as gtk
from gi.repository import AppIndicator3 as appindicator
from gi.repository import Notify as notify

class ROSNetworkCmdLine(object):
    TEXT = 'export UCH_NET_ENABLE='
    def __init__(self, file_name):
        self.file_name = os.path.abspath(file_name)

    def set_state(self, enable=True):
        new_file_list = list()
        with open(self.file_name, 'r') as config_file:
            for raw_line in config_file:
                if ROSNetworkCmdLine.TEXT in raw_line:
                    if enable:
                        new_file_list.append(ROSNetworkCmdLine.TEXT+'true'+os.linesep)
                    else:
                        new_file_list.append(ROSNetworkCmdLine.TEXT+'false'+os.linesep)
                else:
                    new_file_list.append(raw_line)
        with open(self.file_name, 'w') as config_file:
            config_file.writelines(new_file_list)


class ROSNetworkIndicator(ROSNetworkCmdLine):

    APPINDICATOR_ID = 'ros_network_indicator'
    ICON_ENABLE = os.path.join(os.path.dirname(os.path.abspath(__file__)),'ros.png')
    ICON_DISABLE = os.path.join(os.path.dirname(os.path.abspath(__file__)),'ros_red.png')

    def __init__(self, file_name):
        super(ROSNetworkIndicator, self).__init__(file_name)
        # Thread stuff
        self.thread = Thread(target=self.__loop)
        self.lock = RLock()
        self.is_kill = False
        # Indicator
        self.indicator = appindicator.Indicator.new(
            ROSNetworkIndicator.APPINDICATOR_ID,
            ROSNetworkIndicator.ICON_ENABLE,
            appindicator.IndicatorCategory.SYSTEM_SERVICES
        )
        self.indicator.set_status(appindicator.IndicatorStatus.ACTIVE)
        # Menu
        self.menu = gtk.Menu()
        # Enable option
        self.item_enable = gtk.MenuItem('Enable ROS network')
        self.item_enable.connect('activate', self.enable_network)
        self.menu.append(self.item_enable)
        # Quit option
        sep = gtk.SeparatorMenuItem()
        self.menu.append(sep)
        self.menu_item_quit = gtk.ImageMenuItem.new_from_stock(gtk.STOCK_QUIT, None)
        self.menu_item_quit.connect('activate', self.quit)
        self.menu_item_quit.set_always_show_image(True)
        self.menu.append(self.menu_item_quit)
        # Enable indicator
        self.menu.show_all()
        self.indicator.set_menu(self.menu)
        # Notifications
        notify.init(ROSNetworkIndicator.APPINDICATOR_ID)

        self.state = False
        self.update_state()
        self.alert(self.state)
        
    def enable_network(self, source):
        self.set_state(not self.state)
        self.update_state()
        self.alert(self.state)

    def update_state(self):
        with self.lock:
            if self.is_kill:
                return
            self.state = self.get_current_state()
            if self.state:
                self.indicator.set_icon(ROSNetworkIndicator.ICON_ENABLE)
                self.item_enable.set_label('Disable ROS network')
            else:
                self.indicator.set_icon(ROSNetworkIndicator.ICON_DISABLE)
                self.item_enable.set_label('Enable ROS network')


    def get_current_state(self):
        with open(self.file_name, 'r') as config_file:
            for raw_line in config_file:
                if ROSNetworkIndicator.TEXT in raw_line:
                    index = raw_line.find('=')
                    if index == -1:
                        return False
                    return raw_line[index+1:].strip() in ['true', 'True']
        return False

    def run(self):
        # Start thread
        self.thread.start()
        # Run main GTK Loop
        gtk.main()
        # Kill thread
        with self.lock:
            self.is_kill = True

    def quit(self, sorce):
        notify.uninit()
        gtk.main_quit()

    def alert(self, state):
        msg = 'ROS network enabled'
        if not state:
            msg = 'ROS network disabled'
        notify.Notification.new(
            msg,
            'You must open a new terminal for use this configuration.',
            None
        ).show()

    def __loop(self):
        while True:
            if self.is_kill:
                return
            self.update_state()
            sleep(5.0)

def run_appindicator(file_name):
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    app = ROSNetworkIndicator(file_name)
    app.run()

def run_cmdline(file_name, enable=True):
    app = ROSNetworkCmdLine(file_name)
    app.set_state(enable)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('file', help='Configuration file')
    parser.add_argument('--indicator', help='Use GTK indicator', action='store_true')
    
    feature_parser = parser.add_mutually_exclusive_group(required=False)
    feature_parser.add_argument('--enable', help='Enable network', action='store_true')
    feature_parser.add_argument('--disable', help='Disable network', action='store_true')
    
    args = parser.parse_args()
    
    if args.indicator:
        run_appindicator(args.file)
    elif args.enable:
        run_cmdline(args.file, True)
    elif args.disable:
        run_cmdline(args.file, False)
    else:
        parser.print_help()
        sys.exit(1)
