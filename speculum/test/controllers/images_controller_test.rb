require "test_helper"
require "tmpdir"

class ImagesControllerTest < ActionDispatch::IntegrationTest
  test "deletes image names with extensions from query params" do
    with_image_project do |project_root|
      delete delete_image_path(folder: "IMG", name: "delete-me.png")

      assert_redirected_to root_path
      assert_not project_root.join("IMG/delete-me.png").exist?
    end
  end

  test "deletes image names when legacy path format split the extension" do
    with_image_project do |project_root|
      delete "/images/delete-me.png", params: { folder: "IMG" }

      assert_redirected_to root_path
      assert_not project_root.join("IMG/delete-me.png").exist?
    end
  end

  private

  def with_image_project
    Dir.mktmpdir do |project_dir|
      project_root = Pathname.new(project_dir)
      FileUtils.mkdir_p(project_root.join("IMG"))
      project_root.join("IMG/delete-me.png").write("image")
      post session_path, params: { username: "admin", password: "speculum" }

      stub_singleton_method(Speculum::Paths, :project_root, project_root) do
        yield project_root
      end
    end
  end
end
