require "test_helper"
require "tmpdir"

class FoldersControllerTest < ActionDispatch::IntegrationTest
  test "created empty folder is selected and visible in managed order" do
    Dir.mktmpdir do |project_dir|
      Dir.mktmpdir do |storage_dir|
        project_root = Pathname.new(project_dir)
        storage_root = Pathname.new(storage_dir)
        FileUtils.mkdir_p(project_root.join("IMG"))

        stub_singleton_method(Speculum::Paths, :project_root, project_root) do
          stub_singleton_method(Speculum::Paths, :storage_root, storage_root) do
            post session_path, params: { username: "admin", password: "speculum" }
            post folders_path, params: { name: "New Folder" }

            settings = Speculum::Settings.load
            library = Speculum::ImageLibrary.new(settings)

            assert_redirected_to root_path
            assert_path_exists project_root.join("New Folder")
            assert_equal "New Folder", settings["selected_folder"]
            assert_includes library.ordered_folder_names, "New Folder"
          end
        end
      end
    end
  end
end
