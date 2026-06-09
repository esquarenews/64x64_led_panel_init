Rails.application.routes.draw do
  get "up" => "rails/health#show", as: :rails_health_check

  resource :session, only: %i[new create destroy]

  root "dashboard#show"
  patch "settings", to: "dashboard#update", as: :settings
  post "player/start", to: "dashboard#start", as: :start_player
  post "player/pause", to: "dashboard#pause", as: :pause_player
  post "player/reset", to: "dashboard#reset", as: :reset_player

  resources :folders, only: %i[create update destroy] do
    member do
      post :move_up
      post :move_down
    end
  end

  delete "images", to: "images#destroy", as: :delete_image
  delete "images/:name", to: "images#destroy"

  resources :images, only: %i[create], param: :name do
    collection do
      get :show_file
      get :thumbnail
      post :queue
    end
  end
end
