module Speculum
  module Paths
    module_function

    def project_root
      Rails.root.parent
    end

    def image_player
      project_root.join("src/image_player.rb")
    end

    def storage_root
      Rails.root.join("storage")
    end

    def settings_file
      storage_root.join("speculum_settings.yml")
    end

    def runtime_root
      storage_root.join("runtime")
    end

    def single_image_dir
      runtime_root.join("single_image")
    end

    def thumbnail_root
      storage_root.join("thumbnails")
    end

    def queue_file
      runtime_root.join("next_image.txt")
    end

    def pidfile
      runtime_root.join("speculum.pid")
    end

    def logfile
      runtime_root.join("speculum.log")
    end
  end
end
