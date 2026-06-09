module Speculum
  class PortResetter
    def reset_all
      require "serialport"
      ports = PortScanner.candidates
      raise "No USB serial ports found" if ports.empty?

      ports.each { |port| reset_port(port) }
    end

    private

    def reset_port(port)
      serial = SerialPort.new(port, 115_200)
      serial.dtr = 0 if serial.respond_to?(:dtr=)
      serial.rts = 1 if serial.respond_to?(:rts=)
      sleep 0.12
      serial.dtr = 1 if serial.respond_to?(:dtr=)
      serial.rts = 1 if serial.respond_to?(:rts=)
    ensure
      serial&.close
    end
  end
end
