class ApplicationController < ActionController::Base
  allow_browser versions: :modern
  stale_when_importmap_changes

  rescue_from ActionController::InvalidAuthenticityToken, with: :handle_invalid_authenticity_token

  before_action :require_login
  helper_method :current_user_name

  private

  def handle_invalid_authenticity_token
    respond_to do |format|
      format.html { redirect_to(new_session_path, alert: "The page expired. Please sign in again.") }
      format.json { render json: { error: "page_expired" }, status: :unprocessable_entity }
    end
  end

  def require_login
    redirect_to new_session_path unless session[:authenticated]
  end

  def current_user_name
    ENV.fetch("SPECULUM_USERNAME", "admin")
  end
end
