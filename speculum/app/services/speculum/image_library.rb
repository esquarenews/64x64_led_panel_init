require "fileutils"

module Speculum
  class ImageLibrary
    SUPPORTED_EXTENSIONS = %w[.png .jpg .jpeg].freeze
    SAFE_NAME = /\A[[:alnum:]][[:alnum:] ._\-()]*\z/

    attr_reader :settings

    def initialize(settings)
      @settings = settings
    end

    def folders
      ordered_folder_names.map do |name|
        {
          name: name,
          count: images(name).count,
          selected: name == settings["selected_folder"]
        }
      end
    end

    def ordered_folder_names
      names = discovered_folder_names
      ordered = Array(settings["folder_order"]).select { |name| names.include?(name) }
      ordered + (names - ordered)
    end

    def images(folder)
      folder_path(folder).children
                         .select { |path| path.file? && SUPPORTED_EXTENSIONS.include?(path.extname.downcase) }
                         .sort_by { |path| path.basename.to_s.downcase }
                         .map { |path| image_record(folder, path) }
    rescue Errno::ENOENT
      []
    end

    def images_page(folder, page:, per_page:)
      page = [page.to_i, 1].max
      per_page = [per_page.to_i, 1].max
      all = images(folder)
      offset = (page - 1) * per_page
      {
        records: all.slice(offset, per_page) || [],
        total: all.length,
        page: page,
        per_page: per_page,
        total_pages: [(all.length.to_f / per_page).ceil, 1].max
      }
    end

    def image_path(folder, name)
      safe_folder = assert_safe_name(folder, "folder")
      safe_name = assert_safe_filename(name)
      path = folder_path(safe_folder).join(safe_name).expand_path
      raise "Image is outside the project image folder" unless path.to_s.start_with?(folder_path(safe_folder).expand_path.to_s)
      raise "Image not found" unless path.file?

      path
    end

    def upload(folder, upload)
      target_folder = folder_path(assert_safe_name(folder, "folder"))
      FileUtils.mkdir_p(target_folder)
      filename = assert_safe_filename(upload.original_filename)
      ext = File.extname(filename).downcase
      raise "Unsupported image type" unless SUPPORTED_EXTENSIONS.include?(ext)

      target = target_folder.join(filename)
      FileUtils.cp(upload.tempfile.path, target)
      Speculum::Thumbnailer.new.warm(target)
    end

    def delete_image(folder, name)
      FileUtils.rm(image_path(folder, name))
    end

    def queue_image(folder, name)
      path = image_path(folder, name)
      FileUtils.mkdir_p(Paths.runtime_root)
      Paths.queue_file.write(path.basename.to_s)
    end

    def create_folder(name)
      FileUtils.mkdir_p(folder_path(assert_safe_name(name, "folder")))
    end

    def rename_folder(old_name, new_name)
      from = folder_path(assert_safe_name(old_name, "folder"))
      to = folder_path(assert_safe_name(new_name, "folder"))
      raise "Folder not found" unless from.directory?
      raise "Folder already exists" if to.exist?

      FileUtils.mv(from, to)
    end

    def delete_folder(name)
      safe_name = assert_safe_name(name, "folder")
      raise "Folder is not managed by Speculum" unless ordered_folder_names.include?(safe_name)

      path = folder_path(safe_name).expand_path
      raise "Folder is outside the project image area" unless path.to_s.start_with?(Paths.project_root.expand_path.to_s + File::SEPARATOR)
      raise "Folder not found" unless path.directory?

      FileUtils.rm_rf(path)
    end

    def selected_image_path
      if settings["mode"] == "single" && settings["selected_image"].present?
        image_path(settings["selected_folder"], settings["selected_image"])
      end
    rescue StandardError
      nil
    end

    private

    def discovered_folder_names
      Speculum::Paths.project_root.children
                      .select(&:directory?)
                      .map { |path| path.basename.to_s }
                      .select { |name| safe_name?(name) }
                      .select { |name| name.start_with?("IMG") || images(name).any? }
                      .sort_by(&:downcase)
    end

    def folder_path(name)
      Speculum::Paths.project_root.join(name)
    end

    def image_record(folder, path)
      {
        name: path.basename.to_s,
        folder: folder,
        bytes: path.size,
        url: Rails.application.routes.url_helpers.show_file_images_path(folder: folder, name: path.basename.to_s),
        thumbnail_url: Rails.application.routes.url_helpers.thumbnail_images_path(folder: folder, name: path.basename.to_s)
      }
    end

    def assert_safe_name(value, label)
      name = value.to_s.strip
      raise "Invalid #{label} name" unless safe_name?(name)

      name
    end

    def assert_safe_filename(value)
      name = File.basename(value.to_s.strip)
      raise "Invalid image filename" unless safe_name?(name)

      name
    end

    def safe_name?(name)
      name.present? && !name.start_with?(".") && SAFE_NAME.match?(name) && !name.include?("/")
    end
  end
end
