module Speculum
  class PortScanner
    GLOBS = %w[
      /dev/cu.usb*
      /dev/ttyUSB*
      /dev/ttyACM*
      /dev/serial/by-id/*
    ].freeze

    def self.candidates
      ENV.fetch("SPECULUM_SERIAL_PORTS", "")
         .split(",")
         .map(&:strip)
         .reject(&:empty?)
         .presence || GLOBS.flat_map { |glob| Dir[glob] }.uniq.sort
    end
  end
end
