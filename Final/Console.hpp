#pragma once

#include <SFML/Graphics.hpp>

#include <iomanip>
#include <mutex>
#include <sstream>
#include <unordered_map>

class console
{
public:
	static console& instance()
	{
		static console console;
		return console;
	}

	void init()
	{
		if (!font.loadFromFile("arial.ttf"))
		{
			// error...
		}

		m_text.setFont(font);
		m_text.setCharacterSize(12);
		m_text.setFillColor(sf::Color::White);
	}

	template<typename T>
	void set_param(const std::string& name, const T& value)
	{
		std::stringstream stm;
		stm << std::fixed << std::setprecision(2) << value;

		m_params[name] = stm.str();
		update();
	}

	void print(sf::RenderWindow& window)
	{
		std::lock_guard <decltype(m_lock)> ga{ m_lock };

		window.draw(m_text);
	}

private:
    console()
    {

    }

	void update()
	{
		std::stringstream stm;

		for (auto itr : m_params)
			stm << itr.first + ": " + itr.second << std::endl;

		{
			std::lock_guard <decltype(m_lock)> ga{ m_lock };
			m_text.setString(stm.str());
		}
	}

	std::mutex m_lock;
	sf::Text m_text;
	std::unordered_map<std::string, std::string> m_params;
	sf::Font font;
};

template<class SmootherType, class FuncType>
void measure(const std::string name, SmootherType& smoother, const FuncType& func)
{
    using namespace std::chrono;
    using clock = high_resolution_clock;

    const auto start_frame = clock::now();

    func();

    const auto elapsed = duration_cast<milliseconds>(clock::now() - start_frame);

    smoother.add(static_cast<double>(elapsed.count()));

    console::instance().set_param(name, smoother.get());
};

template<class FuncType, typename = 
    std::enable_if_t<std::is_void_v<std::result_of_t<std::decay_t<FuncType>()>>>>
void measure(const std::string name, const FuncType& func)
{
    using namespace std::chrono;
    using clock = high_resolution_clock;

    const auto start_frame = clock::now();

    func();

    const auto elapsed = duration_cast<milliseconds>(clock::now() - start_frame);

    console::instance().set_param(name, elapsed.count());
};

template<class FuncType, typename = 
    std::enable_if_t<!std::is_void_v<std::result_of_t<std::decay_t<FuncType>()>>>>
auto measure(const std::string name, const FuncType& func)
{
    using namespace std::chrono;
    using clock = high_resolution_clock;

    const auto start_frame = clock::now();

    auto result = func();

    const auto elapsed = duration_cast<milliseconds>(clock::now() - start_frame);

    console::instance().set_param(name, elapsed.count());

    return result;
};