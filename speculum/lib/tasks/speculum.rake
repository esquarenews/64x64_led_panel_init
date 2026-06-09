namespace :speculum do
  namespace :thumbnails do
    desc "Generate missing Speculum image thumbnails outside web requests"
    task warm: :environment do
      settings = Speculum::Settings.load
      library = Speculum::ImageLibrary.new(settings)
      thumbnailer = Speculum::Thumbnailer.new
      count = 0

      library.ordered_folder_names.each do |folder|
        library.images(folder).each do |image|
          begin
            path = library.image_path(folder, image[:name])
            next if thumbnailer.cached_thumbnail_path(path)

            thumbnailer.warm(path)
            count += 1
            puts "Warmed #{folder}/#{image[:name]}"
          rescue StandardError => e
            warn "Skipped #{folder}/#{image[:name]}: #{e.message}"
          end
        end
      end

      puts "Generated #{count} thumbnails"
    end
  end
end
