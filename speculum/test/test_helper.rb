ENV["RAILS_ENV"] ||= "test"
require_relative "../config/environment"
require "rails/test_help"

module ActiveSupport
  class TestCase
    # Run tests in parallel with specified workers
    parallelize(workers: 1)

    def stub_singleton_method(object, name, value)
      original = object.method(name)
      object.define_singleton_method(name) { value }
      yield
    ensure
      object.define_singleton_method(name) { |*args, &block| original.call(*args, &block) }
    end
  end
end
