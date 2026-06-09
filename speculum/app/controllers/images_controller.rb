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

  def thumbnail
    path = library.image_path(params.require(:folder), params.require(:name))
    expires_in 30.days, public: true
    send_file Speculum::Thumbnailer.new.thumbnail_path(path), disposition: "inline"
  rescue StandardError
    head :not_found
  end

  def queue
    library.queue_image(params.require(:folder), params.require(:name))
    redirect_to root_path, notice: "#{params[:name]} queued next"
  rescue StandardError => e
    redirect_to root_path, alert: e.message
  end

  private

  def library
    @library ||= Speculum::ImageLibrary.new(Speculum::Settings.load)
  end
end
