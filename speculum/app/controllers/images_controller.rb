class ImagesController < ApplicationController
  def create
    library.upload(params.require(:folder), params.require(:image))
    redirect_to root_path, notice: "Image uploaded"
  rescue StandardError => e
    redirect_to root_path, alert: e.message
  end

  def destroy
    library.delete_image(params.require(:folder), params[:name])
    redirect_to root_path, notice: "Image deleted"
  rescue StandardError => e
    redirect_to root_path, alert: e.message
  end

  def show_file
    path = library.image_path(params.require(:folder), params.require(:name))
    send_file path, disposition: "inline"
  rescue StandardError
    head :not_found
  end

  private

  def library
    @library ||= Speculum::ImageLibrary.new(Speculum::Settings.load)
  end
end
