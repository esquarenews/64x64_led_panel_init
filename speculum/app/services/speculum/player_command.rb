require "fileutils"
require "rbconfig"

module Speculum
  class PlayerCommand
    BOOLEAN_FLAGS = {
      "dither" => "--dither",
      "manual" => "--manual"
    }.freeze

    NEGATED_FLAGS = {
      "color_correct" => "--no-color-correct",
      "calendar" => "--no-calendar",
      "overlay" => "--no-overlay"
    }.freeze

    VALUE_FLAGS = {
      "dwell" => "--dwell",
      "poll" => "--poll",
      "blur" => "--blur",
      "sharpen" => "--sharpen",
      "gamma" => "--gamma"
    }.freeze

    attr_reader :settings, :library

    def initialize(settings, library: ImageLibrary.new(settings))
      @settings = settings
      @library = library
    end

    def argv
      args = [ruby_executable, Paths.image_player.to_s]
      args.concat(["--transport", settings["transport"].presence || "serial"])

      if settings["transport"] == "serial"
        args.concat(["--port", settings["port"].presence || "auto"])
        args.concat(["--baud", settings["baud"].presence || "115200"])
        args << "--hard-reset" if enabled?(settings["hard_reset_before_start"])
      else
        args.concat(["--ble-name", settings["ble_name"].presence || "LRGPanel"])
      end

      args.concat(["--dir", image_source_dir.to_s])
      args.concat(["--queue-file", Paths.queue_file.to_s]) if settings["mode"] != "single"
      args.concat(["--state-file", Paths.state_file.to_s])
      add_value_flags(args)
      add_boolean_flags(args)
      args << "--single" if settings["mode"] == "single"
      args
    end

    def image_source_dir
      return prepare_single_image_dir if settings["mode"] == "single" && library.selected_image_path

      Paths.project_root.join(settings["selected_folder"].presence || "IMG")
    end

    private

    def ruby_executable
      ENV.fetch("SPECULUM_RUBY", RbConfig.ruby)
    end

    def prepare_single_image_dir
      FileUtils.rm_rf(Paths.single_image_dir)
      FileUtils.mkdir_p(Paths.single_image_dir)
      source = library.selected_image_path
      FileUtils.ln_s(source, Paths.single_image_dir.join(source.basename.to_s))
      Paths.single_image_dir
    end

    def add_value_flags(args)
      VALUE_FLAGS.each do |key, flag|
        value = settings[key].to_s.strip
        args.concat([flag, value]) unless value.empty?
      end
    end

    def add_boolean_flags(args)
      BOOLEAN_FLAGS.each { |key, flag| args << flag if enabled?(settings[key]) }
      NEGATED_FLAGS.each { |key, flag| args << flag unless enabled?(settings[key]) }
    end

    def enabled?(value)
      value == true || value.to_s == "1"
    end
  end
end
