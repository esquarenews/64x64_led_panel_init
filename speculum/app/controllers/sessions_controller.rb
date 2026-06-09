class SessionsController < ApplicationController
  skip_before_action :require_login

  def new
    redirect_to root_path if session[:authenticated]
  end

  def create
    if valid_login?
      session[:authenticated] = true
      redirect_to root_path
    else
      flash.now[:alert] = "Invalid username or password"
      render :new, status: :unprocessable_entity
    end
  end

  def destroy
    reset_session
    redirect_to new_session_path
  end

  private

  def valid_login?
    expected_user = ENV.fetch("SPECULUM_USERNAME", "admin")
    expected_password = ENV.fetch("SPECULUM_PASSWORD", "speculum")

    ActiveSupport::SecurityUtils.secure_compare(params[:username].to_s, expected_user) &
      ActiveSupport::SecurityUtils.secure_compare(params[:password].to_s, expected_password)
  rescue ArgumentError
    false
  end
end
