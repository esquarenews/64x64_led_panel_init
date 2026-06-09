class ApplicationController < ActionController::Base
  allow_browser versions: :modern
  stale_when_importmap_changes

  before_action :require_login
  helper_method :current_user_name

  private

  def require_login
    redirect_to new_session_path unless session[:authenticated]
  end

  def current_user_name
    ENV.fetch("SPECULUM_USERNAME", "admin")
  end
end
