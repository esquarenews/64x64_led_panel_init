require "yaml"

module Speculum
  class Settings
    DEFAULTS = {
      "selected_folder" => "IMG",
      "mode" => "loop",
      "selected_image" => nil,
      "transport" => "serial",
      "port" => "auto",
      "baud" => "115200",
      "ble_name" => "LRGPanel",
      "dwell" => "60",
      "poll" => "5",
      "blur" => "",
      "sharpen" => "",
      "gamma" => "2.2",
      "dither" => "0",
      "manual" => "0",
      "color_correct" => "1",
      "calendar" => "1",
      "overlay" => "1",
      "hard_reset_before_start" => "1",
      "folder_order" => []
    }.freeze

    def self.load
      FileUtils.mkdir_p(Paths.storage_root)
      stored = if Paths.settings_file.exist?
        YAML.safe_load(Paths.settings_file.read, permitted_classes: [], aliases: false) || {}
      else
        {}
      end
      DEFAULTS.merge(stored.transform_keys(&:to_s))
    end

    def self.save(settings)
      FileUtils.mkdir_p(Paths.storage_root)
      Paths.settings_file.write(DEFAULTS.merge(settings.transform_keys(&:to_s)).to_yaml)
    end
  end
end
