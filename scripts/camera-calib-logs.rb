#! /usr/bin/env ruby

require 'orocos'
require 'orocos/log'
require 'transformer'
require 'vizkit'
require 'utilrb'
require 'eigen'

include Orocos
Orocos::CORBA.max_message_size = 9000000000

if ARGV.size < 1 then 
    puts "usage: camera-calib-logs.rb <data_log_directory>"
    exit
end

#Initializes the CORBA communication layer
Orocos.initialize

Orocos.run 'camera_calib::Task' => 'camera_calibration' do

    #Config file
    Orocos.conf.load_dir('../config/')

    # get the task
    camera_calib = Orocos.name_service.get 'camera_calibration'
    Orocos.conf.apply(camera_calib, ['default'], :override => true )

    # connect the tasks to the logs
    log_replay = Orocos::Log::Replay.open( ARGV[0] )

    #Connect the log ports to the task
    log_replay.stereo_camera_firewire.frame_left.connect_to(camera_calib.left_frame, :type => :buffer, :size => 100 )
    log_replay.stereo_camera_firewire.frame_right.connect_to(camera_calib.right_frame, :type => :buffer, :size => 100 )

    camera_calib.configure
    camera_calib.start

    # open the log replay widget
    control = Vizkit.control log_replay
    control.speed = 1

    Vizkit.exec

end
