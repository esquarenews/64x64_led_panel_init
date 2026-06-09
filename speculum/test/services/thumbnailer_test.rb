require "test_helper"
require "tmpdir"

class ThumbnailerTest < ActiveSupport::TestCase
  test "falls back to source image when thumbnail generation fails" do
    Dir.mktmpdir do |project_dir|
      source = Pathname.new(project_dir).join("source.png")
      source.write("not an image")

      assert_equal source, Speculum::Thumbnailer.new.thumbnail_path(source)
    end
  end
end
