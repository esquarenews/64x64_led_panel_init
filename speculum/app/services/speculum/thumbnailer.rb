require "digest"
require "fileutils"

module Speculum
  class Thumbnailer
    SIZE = "360x540>".freeze
    PLACEHOLDER_SVG = <<~SVG.freeze
      <svg xmlns="http://www.w3.org/2000/svg" width="360" height="540" viewBox="0 0 360 540">
        <rect width="360" height="540" fill="#151a18"/>
        <rect x="48" y="148" width="264" height="184" rx="8" fill="#26302a"/>
        <circle cx="124" cy="212" r="28" fill="#718257"/>
        <path d="M76 300l76-72 54 48 38-34 40 58z" fill="#dce8d0"/>
        <text x="180" y="384" text-anchor="middle" font-family="system-ui, sans-serif" font-size="18" font-weight="700" fill="#f4f6f3">Thumbnail pending</text>
      </svg>
    SVG

    def thumbnail_path(source)
      source = Pathname.new(source)
      target = Paths.thumbnail_root.join(cache_name(source))
      return target if target.exist?

      warm(source)
    end

    def cached_thumbnail_path(source)
      source = Pathname.new(source)
      target = Paths.thumbnail_root.join(cache_name(source))
      target if target.exist?
    end

    def warm(source)
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

      temp = target.sub_ext(".tmp.jpg")
      image = MiniMagick::Image.open(source.to_s)
      image.auto_orient
      image.resize SIZE
      image.format "jpg"
      image.quality "82"
      image.write temp.to_s
      FileUtils.mv(temp, target)
    ensure
      FileUtils.rm_f(temp) if defined?(temp)
    end
  end
end
