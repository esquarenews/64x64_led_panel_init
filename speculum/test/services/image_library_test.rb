require "test_helper"
require "tmpdir"

class ImageLibraryTest < ActiveSupport::TestCase
  test "discovers safe image folders and image records" do
    Dir.mktmpdir do |project_dir|
      project_root = Pathname.new(project_dir)
      FileUtils.mkdir_p(project_root.join("IMG"))
      FileUtils.mkdir_p(project_root.join(".hidden"))
      project_root.join("IMG/b.png").write("image")
      project_root.join("IMG/a.jpg").write("image")

      stub_singleton_method(Speculum::Paths, :project_root, project_root) do
        library = Speculum::ImageLibrary.new("selected_folder" => "IMG", "folder_order" => [])

        assert_equal ["IMG"], library.ordered_folder_names
        assert_equal ["a.jpg", "b.png"], library.images("IMG").map { |image| image[:name] }
      end
    end
  end

  test "rejects path traversal image names" do
    Dir.mktmpdir do |project_dir|
      project_root = Pathname.new(project_dir)
      FileUtils.mkdir_p(project_root.join("IMG"))

      stub_singleton_method(Speculum::Paths, :project_root, project_root) do
        library = Speculum::ImageLibrary.new("selected_folder" => "IMG")

        assert_raises(RuntimeError) { library.image_path("IMG", "../secret.png") }
      end
    end
  end
end
