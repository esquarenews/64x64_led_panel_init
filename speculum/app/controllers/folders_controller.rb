class FoldersController < ApplicationController
  def create
    library.create_folder(params.require(:name))
    redirect_to root_path, notice: "Folder created"
  rescue StandardError => e
    redirect_to root_path, alert: e.message
  end

  def update
    library.rename_folder(params[:id], params.require(:name))
    settings = Speculum::Settings.load
    settings["selected_folder"] = params[:name] if settings["selected_folder"] == params[:id]
    Speculum::Settings.save(settings)
    redirect_to root_path, notice: "Folder renamed"
  rescue StandardError => e
    redirect_to root_path, alert: e.message
  end

  def destroy
    library.delete_folder(params[:id])
    settings = Speculum::Settings.load
    remaining = Speculum::ImageLibrary.new(settings).ordered_folder_names
    settings["folder_order"] = remaining
    settings["selected_folder"] = remaining.first if settings["selected_folder"] == params[:id]
    Speculum::Settings.save(settings)
    redirect_to root_path, notice: "Folder deleted"
  rescue StandardError => e
    redirect_to root_path, alert: e.message
  end

  def move_up
    move_folder(-1)
  end

  def move_down
    move_folder(1)
  end

  private

  def library
    @library ||= Speculum::ImageLibrary.new(Speculum::Settings.load)
  end

  def move_folder(delta)
    settings = Speculum::Settings.load
    order = Speculum::ImageLibrary.new(settings).ordered_folder_names
    index = order.index(params[:id])
    target = index ? index + delta : nil
    if target && target >= 0 && target < order.length
      order[index], order[target] = order[target], order[index]
      settings["folder_order"] = order
      Speculum::Settings.save(settings)
    end
    redirect_to root_path
  end
end
