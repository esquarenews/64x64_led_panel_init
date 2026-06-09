require "digest"
require "fileutils"

module Speculum
  class Thumbnailer
    SIZE = "360x540>".freeze

    def thumbnail_path(source)
      source = Pathname.new(source)
      FileUtils.mkdir_p(Paths.thumbnail_root)
      target = Paths.thumbnail_root.join(cache_name(source))
      return target if target.exist?

      create_thumbnail(source, target)
      target
    rescue StandardError
      source
    end

    private

    def cache_name(source)
      digest = Digest::SHA256.hexdigest("#{source.expand_path}:#{source.mtime.to_i}:#{source.size}")
      "#{digest}.jpg"
    end

    def create_thumbnail(source, target)
      require "mini_magick"

      image = MiniMagick::Image.open(source.to_s)
      image.auto_orient
      image.resize SIZE
      image.format "jpg"
      image.quality "82"
      image.write target.to_s
    end
  end
end
