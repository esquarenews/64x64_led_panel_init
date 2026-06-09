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
        images = library.images("IMG")
        assert_equal ["a.jpg", "b.png"], images.map { |image| image[:name] }
        assert_includes images.first[:thumbnail_url], "/images/thumbnail"
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

  test "queues an image by filename for the sender" do
    Dir.mktmpdir do |project_dir|
      Dir.mktmpdir do |runtime_dir|
        project_root = Pathname.new(project_dir)
        runtime_root = Pathname.new(runtime_dir)
        FileUtils.mkdir_p(project_root.join("IMG"))
        project_root.join("IMG/queued.png").write("image")

        stub_singleton_method(Speculum::Paths, :project_root, project_root) do
          stub_singleton_method(Speculum::Paths, :runtime_root, runtime_root) do
            stub_singleton_method(Speculum::Paths, :queue_file, runtime_root.join("next_image.txt")) do
              library = Speculum::ImageLibrary.new("selected_folder" => "IMG")
              library.queue_image("IMG", "queued.png")

              assert_equal "queued.png", runtime_root.join("next_image.txt").read
            end
          end
        end
      end
    end
  end

  test "paginates image records to keep dashboard load bounded" do
    Dir.mktmpdir do |project_dir|
      project_root = Pathname.new(project_dir)
      FileUtils.mkdir_p(project_root.join("IMG"))
      5.times { |index| project_root.join("IMG/#{index}.png").write("image") }

      stub_singleton_method(Speculum::Paths, :project_root, project_root) do
        library = Speculum::ImageLibrary.new("selected_folder" => "IMG")
        page = library.images_page("IMG", page: 2, per_page: 2)

        assert_equal 5, page[:total]
        assert page[:has_next]
        assert_equal 2, page[:records].length
        assert_equal ["2.png", "3.png"], page[:records].map { |image| image[:name] }
      end
    end
  end

  test "folder counts do not build full image records" do
    Dir.mktmpdir do |project_dir|
      project_root = Pathname.new(project_dir)
      FileUtils.mkdir_p(project_root.join("IMG"))
      3.times { |index| project_root.join("IMG/#{index}.png").write("image") }

      stub_singleton_method(Speculum::Paths, :project_root, project_root) do
        library = Speculum::ImageLibrary.new("selected_folder" => "IMG")

        assert_equal 3, library.image_count("IMG")
        assert_equal ["0.png", "1.png", "2.png"], library.image_names("IMG")
        assert_equal 3, library.folders.first[:count]
      end
    end
  end

  test "ignores unsupported or unsafe image filenames" do
    Dir.mktmpdir do |project_dir|
      project_root = Pathname.new(project_dir)
      FileUtils.mkdir_p(project_root.join("IMG"))
      project_root.join("IMG/good.png").write("image")
      project_root.join("IMG/bad:name.png").write("image")
      project_root.join("IMG/notes.txt").write("text")

      stub_singleton_method(Speculum::Paths, :project_root, project_root) do
        library = Speculum::ImageLibrary.new("selected_folder" => "IMG")

        assert_equal ["good.png"], library.image_names("IMG")
        assert_equal 1, library.image_count("IMG")
      end
    end
  end

  test "reuses one folder scan across request level image lookups" do
    Dir.mktmpdir do |project_dir|
      project_root = Pathname.new(project_dir)
      FileUtils.mkdir_p(project_root.join("IMG"))
      project_root.join("IMG/alpha.png").write("image")

      stub_singleton_method(Speculum::Paths, :project_root, project_root) do
        library = Speculum::ImageLibrary.new("selected_folder" => "IMG")

        assert_equal ["alpha.png"], library.image_names("IMG")

        project_root.join("IMG/bravo.png").write("image")

        assert_equal ["alpha.png"], library.image_names("IMG")
      end
    end
  end

  test "removes a folder and its images" do
    Dir.mktmpdir do |project_dir|
      project_root = Pathname.new(project_dir)
      FileUtils.mkdir_p(project_root.join("IMG/remove-me"))
      FileUtils.mkdir_p(project_root.join("IMG-old"))
      project_root.join("IMG-old/photo.png").write("image")

      stub_singleton_method(Speculum::Paths, :project_root, project_root) do
        library = Speculum::ImageLibrary.new("selected_folder" => "IMG-old")
        library.delete_folder("IMG-old")

        assert_not project_root.join("IMG-old").exist?
      end
    end
  end

  test "does not remove unmanaged project folders" do
    Dir.mktmpdir do |project_dir|
      project_root = Pathname.new(project_dir)
      FileUtils.mkdir_p(project_root.join("speculum"))

      stub_singleton_method(Speculum::Paths, :project_root, project_root) do
        library = Speculum::ImageLibrary.new("selected_folder" => "speculum")

        assert_raises(RuntimeError) { library.delete_folder("speculum") }
        assert_path_exists project_root.join("speculum")
      end
    end
  end
end
