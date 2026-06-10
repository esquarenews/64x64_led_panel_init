require "bundler"
require "test_helper"

class ImagePlayerDependencyTest < ActiveSupport::TestCase
  test "production bundle includes image player fallback gems" do
    specs = Bundler.load.specs.map(&:name)

    assert_includes specs, "serialport"
    assert_includes specs, "mini_magick"
    assert_includes specs, "chunky_png"
  end

  test "queued image marker is kept until frame acknowledgement" do
    script = Rails.root.parent.join("src/image_player.rb").read
    queued_reader = script[/def queued_image_path.*?^end/m]
    acknowledgement_index = script.index("frames_sent += 1")
    queue_delete_index = script.index("FileUtils.rm_f(options[:queue_file])")

    assert_not_includes queued_reader, "FileUtils.rm_f(queue_file)"
    assert queue_delete_index > acknowledgement_index
  end
end
