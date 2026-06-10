class DashboardController < ApplicationController
  IMAGES_PER_PAGE = 18

  def show
    expires_now
    @settings = Speculum::Settings.load
    @library = Speculum::ImageLibrary.new(@settings)
    @player = Speculum::ProcessManager.new
    @folders = @library.folders
    @selected_folder = @settings["selected_folder"]
    @image_page = @library.images_page(@selected_folder, page: params[:page], per_page: IMAGES_PER_PAGE)
    @images = @image_page[:records]
    @preview = @player.preview(@library, @settings)
    @ports = Speculum::PortScanner.candidates
  end

  def update
    settings = Speculum::Settings.load
    permitted = settings_params
    folder_changed = permitted.key?("selected_folder") && permitted["selected_folder"] != settings["selected_folder"]
    settings.merge!(permitted)
    settings["folder_order"] = Speculum::ImageLibrary.new(settings).ordered_folder_names
    Speculum::Settings.save(settings)

    player = Speculum::ProcessManager.new
    if folder_changed && player.running?
      player.restart(settings.merge("hard_reset_before_start" => "1"))
      redirect_to root_path, notice: "Settings saved and panel reset for folder change"
    else
      redirect_to root_path, notice: "Settings saved"
    end
  rescue StandardError => e
    redirect_to root_path, alert: e.message
  end

  def start
    settings = Speculum::Settings.load
    result = Speculum::ProcessManager.new.start(settings)
    message = result == :restarted ? "Speculum recovered and restarted" : "Speculum started"
    redirect_to root_path, notice: message
  rescue StandardError => e
    redirect_to root_path, alert: e.message
  end

  def pause
    Speculum::ProcessManager.new.pause
    redirect_to root_path, notice: "Speculum paused"
  end

  def reset
    Speculum::PortResetter.new.reset_all
    player = Speculum::ProcessManager.new
    settings = Speculum::Settings.load
    player.restart(settings.merge("hard_reset_before_start" => "1"))
    redirect_to root_path, notice: "Panel reset and Speculum restarted"
  rescue StandardError => e
    redirect_to root_path, alert: e.message
  end

  def preview
    expires_now
    settings = Speculum::Settings.load
    library = Speculum::ImageLibrary.new(settings)
    player = Speculum::ProcessManager.new
    render json: {
      running: player.running?,
      preview: preview_payload(player.preview(library, settings))
    }
  end

  private

  def preview_payload(preview)
    {
      current: image_payload(preview[:current]),
      next: image_payload(preview[:next]),
      timer: preview[:timer]
    }
  end

  def image_payload(image)
    return unless image

    {
      name: image[:name],
      thumbnail_url: image[:thumbnail_url]
    }
  end

  def settings_params
    permitted = params.require(:settings).permit(
      :selected_folder, :mode, :selected_image, :transport, :port, :baud, :dwell,
      :poll, :blur, :sharpen, :gamma, :overlay_text, :ble_name,
      :dither, :manual, :color_correct, :calendar, :overlay, :hard_reset_before_start
    ).to_h

    %w[dither manual color_correct calendar overlay hard_reset_before_start].each do |key|
      permitted[key] = params[:settings][key] if params[:settings].key?(key)
    end

    permitted
  end
end
