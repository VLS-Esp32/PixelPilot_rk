
extern "C" {
#include "drm.h"
#include "mavlink.h"
}
#include "osd.h"
#include "osd.hpp"

#include <pthread.h>
#include <map>
#include <vector>
#include <ranges>
#include <memory>
#include <variant>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <deque>
#include <cstdlib> //KILLME
#include <string>
#include <filesystem>
#include <cairo.h>
#include <nlohmann/json.hpp>
#include "spdlog/spdlog.h"
#include <fmt/ranges.h>

#define WFB_LINK_LOST 1
#define WFB_LINK_JAMMED 2

#define PATH_MAX	4096

using json = nlohmann::json;

bool enable_osd = false;
int osd_zpos = 2;
extern uint32_t refresh_frequency_ms;
extern uint32_t frames_received;
uint32_t stats_rx_bytes = 0;
struct timespec last_timestamp = {0, 0};
float rx_rate = 0;
int hours = 0, minutes = 0, seconds = 0, milliseconds = 0;
char custom_msg[80];
u_int custom_msg_refresh_count = 0;
extern pthread_mutex_t video_mutex;
extern pthread_cond_t video_cond;
bool osd_update_ready = false;
extern std::atomic<bool> video_present;

struct Size {
    int width = 0;
    int height = 0;
};

struct Position {
    int x = 0;
    int y = 0;
};

struct CairoColor {
    double r;
    double g;
    double b;
    double a;
};

struct DrawStyle {
    CairoColor fill;
    CairoColor outline;
    double outline_width;
};


double getTimeInterval(struct timespec* timestamp, struct timespec* last_meansure_timestamp) {
  return (timestamp->tv_sec - last_meansure_timestamp->tv_sec) +
       (timestamp->tv_nsec - last_meansure_timestamp->tv_nsec) / 1000000000.;
}

//
// Facts
//

typedef std::map<std::string, std::string> FactTags;


class FactMatcher {
public:
	FactMatcher(std::string name, FactTags tags): name(name), tags(tags) {};
	FactMatcher(std::string name): name(name), tags({}) {};
	std::string name;
	FactTags tags;
};


class FactMeta {
public:
	FactMeta(): name(""), tags({}) {};
	FactMeta(std::string name): name(name), tags({}) {};
	FactMeta(std::string name, FactTags tags): name(name), tags(tags) {};


	std::string getName() { return name; }
	FactTags getTags() { return tags; }

	/**
	 * Returns true if names are equal and all match_tags are defined and have equal value
	 */
	bool match(FactMatcher matcher) {
		if(matcher.name != name) return false;
		for (const auto& [key, match_value] : matcher.tags) {
			if (auto value = tags.find(key); value != tags.end()) {
				if (value->second != match_value) return false;
			} else {
				return false;
			}
		}
		return true;
	}

private:
	std::string name;
	FactTags tags;
};


class Fact {
public:
	enum Type {
		T_UNDEF,
		T_BOOL,
		T_INT,
		T_UINT,
		T_DOUBLE,
		T_STRING
	};

	Fact(): meta(FactMeta("", {})), type(T_UNDEF) {};
	Fact(FactMeta meta, bool val): meta(meta), value(val), type(T_BOOL) {};
	Fact(FactMeta meta, long val): meta(meta), value(val), type(T_INT) {};
	Fact(FactMeta meta, ulong val): meta(meta), value(val), type(T_UINT) {};
	Fact(FactMeta meta, double val): meta(meta), value(val), type(T_DOUBLE) {};
	Fact(FactMeta meta, std::string val): meta(meta), value(val), type(T_STRING) {};
	Fact(FactMeta meta): meta(meta), type(T_UNDEF) {};

	bool isDefined() {
		return type != T_UNDEF;
	}

	// TODO: try to cast instead of crash
	bool getBoolValue() {
		assertType(T_BOOL);
		return std::get<bool>(value);
	}

	long getIntValue() {
		assertType(T_INT);
		return std::get<long>(value);
	}

	ulong getUintValue() {
		assertType(T_UINT);
		return std::get<ulong>(value);
	}

	double getDoubleValue() {
		assertType(T_DOUBLE);
		return std::get<double>(value);
	}

	std::string getStrValue() {
		assertType(T_STRING);
		return std::get<std::string>(value);
	}

	bool matches(FactMatcher matcher) {
		return meta.match(matcher);
	}

	std::string getTypeName() {
		return typeName(type);
	}

	Type getType() {
		return type;
	}

	std::string getName() {
		return meta.getName();
	}

	FactTags getTags() {
		return meta.getTags();
	}

	std::string asString() {
		switch(type) {
		case T_UNDEF:
			return "(undefined)";
		case T_BOOL:
			if (getBoolValue()) {
				return "true";
			} else {
				return "false";
			};
		case T_INT:
			return std::to_string(getIntValue());
		case T_UINT:
			return std::to_string(getUintValue());
		case T_DOUBLE:
			return std::to_string(getDoubleValue());
		case T_STRING:
			return getStrValue();
		}
		return "(unknown)";
	}
	
private:
	Type type = T_UNDEF;
	std::string typeName(Type t) {
		switch(t) {
		case T_UNDEF:
			return "UNDEF";
		case T_BOOL:
			return "BOOL";
		case T_INT:
			return "INT";
		case T_UINT:
			return "UINT";
		case T_DOUBLE:
			return "DOUBLE";
		case T_STRING:
			return "STRING";
		}
		return "UNKNOWN";
	}

	void assertType(Type t) {
		if (t != type) {
			spdlog::error("'{}': requested type of {}, but the actual type is {}",
						  meta.getName(), typeName(t), typeName(type));
			assert(type == t);
		}
	}
	FactMeta meta;
	// TODO: timestamp
	std::variant<
		bool,
		long,
		ulong,
		double,
		std::string
		> value;
};



struct Bucket {
	long long timestamp;
	long sum;
	int count;
	long min_value;
	long max_value;

	Bucket(long long ts, long value)
		: timestamp(ts), sum(value), count(1), min_value(value), max_value(value) {}
};

// Struct to hold extended statistics
struct Stats {
	long min;
	long max;
	double average;
	long sum;
	int count;

	Stats(long min_value, long max_value, double avg, long total_sum, int total_count)
		: min(min_value), max(max_value), average(avg), sum(total_sum), count(total_count) {}
};

/**
 * Calculates the running average/rate-per-second/min/max over a sliding time window.
 * @param window_size_ms the size of the sliding window in milliseconds
 * @param bucket_size_ms the size of the bucket; structure uses amount of memory
 *		  of O(window_size_ms / bucket_size_ms), however large bucket size decreases the precision.
 * NOTE: the code was mostly generated by ChatGPT
 */
class RunningAverage {
public:
	RunningAverage(int window_size_ms, int bucket_size_ms)
		: window_size(window_size_ms), bucket_size(bucket_size_ms), sum(0), count(0) {
		assert(window_size_ms >= bucket_size_ms);
	}

	long add(long value) {
		auto now = std::chrono::steady_clock::now();
		auto current_time = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();

		// Remove outdated buckets
		while (!buckets.empty() && (current_time - buckets.front().timestamp > window_size)) {
			sum -= buckets.front().sum;
			count -= buckets.front().count;
			buckets.pop_front();
		}

		// Add the value to the current bucket
		if (!buckets.empty() && (current_time - buckets.back().timestamp < bucket_size)) {
			buckets.back().sum += value;
			buckets.back().count += 1;
			buckets.back().min_value = std::min(buckets.back().min_value, value);
			buckets.back().max_value = std::max(buckets.back().max_value, value);
		} else {
			buckets.emplace_back(current_time, value);
		}

		// Update the running sum and count
		sum += value;
		count++;

		return count > 0 ? sum / count : 0;
	}

	double average_over_last_ms(uint last_ms) const {
		long min = std::numeric_limits<long>::max();
		long max = std::numeric_limits<long>::min();
		long last_sum;
		int last_count;
		calculate_stats_in_window(last_ms, last_sum, last_count, min, max);

		return last_count > 0 ? static_cast<double>(last_sum) / last_count : 0.0;
	}

	double rate_per_second_over_last_ms(uint last_ms) const {
		long min = std::numeric_limits<long>::max();
		long max = std::numeric_limits<long>::min();
		long last_sum;
		int last_count;
		calculate_stats_in_window(last_ms, last_sum, last_count, min, max);

		double elapsed_seconds = static_cast<double>(last_ms) / 1000.0;
		return elapsed_seconds > 0 ? static_cast<double>(last_sum) / elapsed_seconds : 0.0;
	}

	void get_stats_over_last_ms(uint last_ms, long& min, long& max, double& average) const {
		long last_sum;
		int last_count;

		min = std::numeric_limits<long>::max();
		max = std::numeric_limits<long>::min();

		calculate_stats_in_window(last_ms, last_sum, last_count, min, max);

		average = last_count > 0 ? static_cast<double>(last_sum) / last_count : 0.0;
	}

	// New method to return Stats struct with sum and count
	Stats get_stats_over_last_ms_result(uint last_ms) const {
		long min = std::numeric_limits<long>::max();
		long max = std::numeric_limits<long>::min();
		long last_sum = 0;
		int last_count = 0;

		calculate_stats_in_window(last_ms, last_sum, last_count, min, max);

		double average = last_count > 0 ? static_cast<double>(last_sum) / last_count : 0.0;
		return Stats(min, max, average, last_sum, last_count);
	}

	std::vector<long> get_bucket_sums() const {
		std::vector<long> sums;
		sums.reserve(buckets.size());
		for (const auto& bucket : buckets) {
			sums.push_back(bucket.sum);
		}
		return sums;
	}

	std::vector<Stats> get_bucket_stats() const {
		std::vector<Stats> stats;
		stats.reserve(buckets.size());
		for (const auto& bucket : buckets) {
			double average = bucket.count > 0 ? static_cast<double>(bucket.sum) / bucket.count : 0.0;
			stats.push_back(Stats(bucket.min_value, bucket.max_value, average, bucket.sum, bucket.count));
		}
		return stats;
	}

private:
	void calculate_stats_in_window(uint last_ms, long& sum_out, int& count_out, long& min_out, long& max_out) const {
		auto now = std::chrono::steady_clock::now();
		auto current_time = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();

		sum_out = 0;
		count_out = 0;

		for (auto it = buckets.rbegin(); it != buckets.rend(); ++it) {
			if (current_time - it->timestamp <= last_ms) {
				sum_out += it->sum;
				count_out += it->count;
				min_out = std::min(min_out, it->min_value);
				max_out = std::max(max_out, it->max_value);
			} else {
				break;	// Exit loop once we're outside the time window
			}
		}
	}

	int window_size;
	int bucket_size;
	std::deque<Bucket> buckets;
	long sum;
	int count;
};

//
// Widgets
//

class Widget {
public:
    Widget(int pos_x, int pos_y, uint num_args = 0) : position_{pos_x, pos_y}, args_(num_args) {}
    virtual ~Widget() = default;

	Widget(const Widget&) = delete;
	Widget& operator=(const Widget&) = delete;

    virtual void draw(cairo_t* cr) = 0;
    virtual void measure(cairo_t* cr) = 0;

    virtual void setFact(uint idx, Fact fact) {
        storeFact(idx, std::move(fact));
    }

    void setPosition(int x, int y) {
        position_ = {x, y};
    }

    bool measureDirty() const {
        return measure_dirty_;
    }

    int width() const {
        return size_.width;
    }

    int height() const {
        return size_.height;
    }

    const Size& size() const {
        return size_;
    }

    const Position& position() const {
        return position_;
    }

    int x(cairo_t* cr) const {
        cairo_surface_t* target = cairo_get_target(cr);
        int w = cairo_image_surface_get_width(target);
        return (w + position_.x) % w;
    }

    int y(cairo_t* cr) const {
        cairo_surface_t* target = cairo_get_target(cr);
        int h = cairo_image_surface_get_height(target);
        return (h + position_.y) % h;
    }

    std::pair<int, int> xy(cairo_t *cr) const {
        return {x(cr), y(cr)};
    }

protected:
    void measureChild(Widget& child, cairo_t* cr) {
        if (child.measureDirty()) {
            child.measure(cr);
        }
    }

    void storeFact(uint idx, Fact fact) {
        args_.at(idx) = std::move(fact);
        measure_dirty_ = true;
    }

    const Fact& fact(uint idx) const {
        return args_.at(idx);
    }

    uint factCount() const {
        return static_cast<uint>(args_.size());
    }

    void invalidateMeasure() {
        measure_dirty_ = true;
    }

    void setSize(int width, int height) {
        size_ = {width, height};
        measure_dirty_ = false;
    }

private:
    Size size_;
    Position position_;
    bool measure_dirty_ = true;

    std::vector<Fact> args_;
};

class TextWidget: public Widget {
public:
    TextWidget(int pos_x, int pos_y, std::string text, uint num_args = 0, DrawStyle style = DEFAULT_STYLE):
        Widget(pos_x, pos_y, num_args), text_(std::move(text)), style_(style) {}

    void setText(std::string text) {
        if (text_ == text)
            return;
        text_ = std::move(text);
        invalidateMeasure();
    }

    const std::string& text() const {
        return text_;
    }

    void draw(cairo_t* cr) override {
        auto [x, y] = xy(cr);
        drawAt(cr, x, y);
    }

    void drawAt(cairo_t *cr, double x, double y) const {
        drawText(cr, x, y);
    }

    void measure(cairo_t* cr) override {
        cairo_save(cr);
        buildTextPath(cr, 0.0, 0.0);

        double x1, y1, x2, y2;
        if (style_.outline_width > 0.0) {
            setupStroke(cr);
            cairo_stroke_extents(cr, &x1, &y1, &x2, &y2);
        } else {
            cairo_fill_extents(cr, &x1, &y1, &x2, &y2);
        }
        setSize(static_cast<int>(std::ceil(x2 - x1)), static_cast<int>(std::ceil(y2 - y1)));

        cairo_new_path(cr);
        cairo_restore(cr);
    }

	void setFillColor(const CairoColor& color) {
        style_.fill = color;
    }

    void setOutlineColor(const CairoColor& color) {
        style_.outline = color;
    }

    void setOutlineWidth(double width) {
        if (style_.outline_width == width)
            return;
        style_.outline_width = width;
        invalidateMeasure();
    }

    void setStyle(const DrawStyle& style) {
        if (style_.outline_width != style.outline_width)
            invalidateMeasure();
        style_ = style;
    }

    const DrawStyle& style() const {
        return style_;
    }

protected:
    void drawText(cairo_t* cr, double x, double y) const {
        cairo_save(cr);
        buildTextPath(cr, x, y);

        if (style_.outline_width > 0.0) {
            setupStroke(cr);
            cairo_set_source_rgba(cr, style_.outline.r, style_.outline.g, style_.outline.b, style_.outline.a);
            cairo_stroke_preserve(cr);
        }
        cairo_set_source_rgba(cr, style_.fill.r, style_.fill.g, style_.fill.b, style_.fill.a);
        cairo_fill(cr);
        cairo_restore(cr);
    }

private:
    void buildTextPath(cairo_t* cr, double x, double y) const {
        cairo_new_path(cr);
        cairo_move_to(cr, x, y);
        cairo_text_path(cr, text_.c_str());
    }

    void setupStroke(cairo_t* cr) const {
        cairo_set_line_width(cr, style_.outline_width);
        cairo_set_line_join(cr, CAIRO_LINE_JOIN_ROUND);
    }

	static constexpr DrawStyle DEFAULT_STYLE{
		.fill = {1.0, 1.0, 1.0, 1.0},
		.outline = {0.0, 0.0, 0.0, 1.0},
		.outline_width = 2.0
	};

    std::string text_;
	DrawStyle style_;
};


class IconWidget: public Widget {
public:
    IconWidget(int pos_x, int pos_y, cairo_surface_t* icon, uint num_args = 0, DrawStyle style = DEFAULT_STYLE):
		Widget(pos_x, pos_y, num_args), icon_(icon), style_(style) {}

    ~IconWidget() override {
        if (icon_)
            cairo_surface_destroy(icon_);
    }

    void measure(cairo_t*) override {
        if (!icon_) {
            setSize(0, 0);
            return;
        }
        int width = cairo_image_surface_get_width(icon_);
        int height = cairo_image_surface_get_height(icon_);
		int outline = outlineWidth();

        setSize(width + outline * 2, height + outline * 2);
    }

    void draw(cairo_t* cr) override {
        auto [x, y] = xy(cr);
        drawIcon(cr, x, y - 20);
    }

	void drawAt(cairo_t *cr, double x, double y) const {
		drawIcon(cr, x, y);
	}

	void setFillColor(const CairoColor& color) {
        style_.fill = color;
    }

    void setOutlineColor(const CairoColor& color) {
        style_.outline = color;
    }

    void setOutlineWidth(double width) {
        if (style_.outline_width == width)
            return;
        style_.outline_width = width;
        invalidateMeasure();
    }

    void setStyle(const DrawStyle& style) {
        if (style_.outline_width != style.outline_width)
            invalidateMeasure();
        style_ = style;
    }

    const DrawStyle& style() const {
        return style_;
    }

protected:
    cairo_surface_t* icon() const {
        return icon_;
    }

    void drawIcon(cairo_t* cr, double x, double y) const {
        if (!icon_)
            return;

        cairo_save(cr);
		int outline = outlineWidth();

        if (outline > 0) {
            cairo_set_source_rgba(cr, style_.outline.r, style_.outline.g, style_.outline.b, style_.outline.a);
            for (int dx = -outline; dx <= outline; ++dx) {
                for (int dy = -outline; dy <= outline; ++dy) {
                    if (dx * dx + dy * dy > outline * outline)
                        continue;
                    cairo_mask_surface(cr, icon_, x + dx, y + dy);
                }
            }
        }
        cairo_set_source_rgba(cr, style_.fill.r, style_.fill.g, style_.fill.b, style_.fill.a);
        cairo_mask_surface(cr, icon_, x, y);
        cairo_restore(cr);
    }

	int outlineWidth() const {
		return static_cast<int>(std::ceil(style_.outline_width));
	}

private:
	static constexpr DrawStyle DEFAULT_STYLE{
    	.fill = {1.0, 1.0, 1.0, 1.0},
    	.outline = {0.0, 0.0, 0.0, 1.0},
    	.outline_width = 1.0
	};

    cairo_surface_t* icon_;
	DrawStyle style_;
};

class IconTextWidget: public Widget {
public:
    IconTextWidget(int pos_x, int pos_y, cairo_surface_t *icon, std::string text, uint num_args = 0):
		Widget(pos_x, pos_y, num_args), icon_(0, 0, icon), text_(0, 0, std::move(text)) {}

    void measure(cairo_t* cr) override {
        measureChild(icon_, cr);
        measureChild(text_, cr);
        setSize(
            icon_.width() + SPACING + text_.width(),
            std::max(icon_.height(), text_.height())
        );
    }

    void draw(cairo_t* cr) override {
        auto [x, y] = xy(cr);
        icon_.drawAt(cr, x, y - 20);
        text_.drawAt(cr, x + icon_.width() + SPACING, y);
    }

private:
    static constexpr int SPACING = 14;

    IconWidget icon_;
    TextWidget text_;
};


class TplTextWidget: public TextWidget {
public:
	TplTextWidget(int pos_x, int pos_y, std::string tpl, uint num_args):
		TextWidget(pos_x, pos_y, "", num_args), tpl_(std::move(tpl)) {
        setText(renderTpl());
    }

    void measure(cairo_t* cr) override {
        setText(renderTpl());
        TextWidget::measure(cr);
    }

private:
	std::string renderTpl() const {
        std::string msg;
        msg.reserve(tpl_.size());

        uint fact_i = 0;
		for (std::size_t i = 0; i < tpl_.size(); ++i) {
            char c = tpl_[i];
            if (c != '%') {
                msg.push_back(c);
                continue;
            }
            if (i + 1 >= tpl_.size()) {
                msg.push_back('%');
                break;
            }

            char spec = tpl_[++i];
            if (spec == '%') {
                msg.push_back('%');
                continue;
            }
            if (fact_i >= factCount()) {
                msg.push_back('-');
                continue;
            }

            Fact fact = this->fact(fact_i++);
            if (!fact.isDefined()) {
                msg.push_back('-');
                continue;
            }
            switch (spec) {
            case 'b':
                msg.push_back(fact.getBoolValue() ? 't' : 'f');
                break;
            case 'd':
            case 'i':
                msg.append(std::to_string(fact.getIntValue()));
                break;
            case 'u':
                msg.append(std::to_string(fact.getUintValue()));
                break;
            case 'f': {
                char buf[32];
                std::snprintf(buf, sizeof(buf), "%.2f", fact.getDoubleValue());
                msg.append(buf);
                break;
            }
            case 's':
                msg.append(fact.getStrValue());
                break;
            default:
                msg.push_back('-');
                break;
            }
        }
        return msg;
	}

	std::string tpl_;
};


class IconTplTextWidget: public TplTextWidget {
public:
    IconTplTextWidget(int pos_x, int pos_y, cairo_surface_t *icon, std::string tpl, uint num_args):
        TplTextWidget(pos_x, pos_y, std::move(tpl), num_args), icon_(0, 0, icon) {}

    void measure(cairo_t* cr) override {
        measureChild(icon_, cr);
        TplTextWidget::measure(cr);
        setSize(
            icon_.width() + SPACING + TplTextWidget::width(),
            std::max(icon_.height(), TplTextWidget::height())
        );
    }

    void draw(cairo_t* cr) override {
        auto [x, y] = xy(cr);
        icon_.drawAt(cr, x, y - 20);
        drawText(cr, x + icon_.width() + SPACING, y);
    }

private:
    static constexpr int SPACING = 14;
    IconWidget icon_;
};

class BoxWidget: public Widget {
public:
    BoxWidget(int pos_x, int pos_y, uint width, uint height, CairoColor color):
		Widget(pos_x, pos_y), width_(width), height_(height), color_(color) {
        setSize(width_, height_);
    }

    void measure(cairo_t*) override {
        setSize(width_, height_);
    }

    void draw(cairo_t* cr) override {
        auto [x, y] = xy(cr);
        cairo_set_source_rgba(cr, color_.r, color_.g, color_.b, color_.a);
        cairo_rectangle(cr, x, y, width_, height_);
        cairo_fill(cr);
    }

private:
    uint width_;
    uint height_;
    CairoColor color_;
};

class BarChartWidget: public Widget {
public:
	enum StatsField {
		STATS_MIN,
		STATS_MAX,
		STATS_SUM,
		STATS_COUNT,
		STATS_AVG
	};

    BarChartWidget(int pos_x, int pos_y, uint width, uint height, uint window_s, uint num_buckets, StatsField stats_field):
		Widget(pos_x, pos_y, 1), width_(width), height_(height), window_ms_(window_s * 1000), num_buckets_(num_buckets), 
		stats_field_(stats_field), stats_(window_s * 1000, window_s * 1000 / num_buckets), max_label_(0, 0, ""), min_label_(0, 0, "") {
        setSize(width_, height_);
    }

    void measure(cairo_t *) override {
        setSize(width_, height_);
    }

    void draw(cairo_t *cr) override {
        auto [x, y] = xy(cr);
        // box
        cairo_set_source_rgba(cr, 0.0, 0.0, 0.0, 0.4);
        cairo_rectangle(cr, x, y, width_, height_);
        cairo_fill(cr);

        std::vector<Stats> all_stats = stats_.get_bucket_stats();
        if (all_stats.size() < 3) {
            SPDLOG_DEBUG("Can't draw bar chart - too few values");
            return;
        }
        all_stats.pop_back(); // drop last bucket, because it is usually still not full
        std::vector<double> stats = select_stats(all_stats);
        double min = *std::min_element(stats.begin(), stats.end());
        double max = *std::max_element(stats.begin(), stats.end());

        // legend
        max_label_.setText(shorten(max));
        min_label_.setText(shorten(min));
        max_label_.drawAt(cr, x + 2, y + 15);
        min_label_.drawAt(cr, x + 2, y + height_);

        // bars
        cairo_set_source_rgba(cr, 200.0, 200.0, 200.0, 0.8);

        double scale = max - min;
        SPDLOG_TRACE("Scale: {}, min {}, max {}", scale, min, max);
        uint legend_w = 65;
        uint chart_w = width_ - legend_w;

        uint bar_pad = 4;
        uint bar_w = (chart_w - (bar_pad * num_buckets_)) / num_buckets_;
        uint bar_x = x + legend_w;
		SPDLOG_TRACE(
			 "chart_w {} bar_w {}, bar_x {}",
			 chart_w, bar_w, bar_x
            );

        for (auto val : stats) {
            double normalized = val - min;
            double bar_h = -1.0 * (normalized * (height_ - 10)) / scale;
			// h -> max-min
			// ? -> normalized
			SPDLOG_TRACE("val {}, cairo_rectangle(cr, {}, {}, {}, {})",
						 val, bar_x, y + height_, bar_w, bar_h);
            cairo_rectangle(cr, bar_x, y + height_, bar_w, bar_h - 2);
            cairo_fill(cr);
            bar_x += bar_pad + bar_w;
        }
    }

    void setFact(uint idx, Fact fact) override {
        assert(idx == 0);
        switch (fact.getType()) {
        case Fact::T_INT:
            stats_.add(fact.getIntValue());
            break;
        case Fact::T_UINT:
            stats_.add(static_cast<long>(fact.getUintValue()));
            break;
        }
    }

private:
	/**
	 * function that takes ulong and returns string with short form of the number:
	 * up to 3 digits and "giga" / "mega" / "kilo" suffix
	 * made by ChatGPT
	 */
	std::string shorten(long num) {
		double value = num;
		std::string suffix;

		if (num >= 1'000'000'000) {  // Giga
			value = num / 1'000'000'000.0;
			suffix = "G";
		} else if (num >= 1'000'000) {  // Mega
			value = num / 1'000'000.0;
			suffix = "M";
		} else if (num >= 1'000) {  // Kilo
			value = num / 1'000.0;
			suffix = "K";
		} else {
			suffix = "";  // No suffix needed
		}

		// Format to 3 significant digits
		std::ostringstream oss;
		oss << std::fixed << std::setprecision(3 - static_cast<int>(std::log10(value) + 1)) << value;
		return oss.str() + " " + suffix;
	}
	std::vector<double> select_stats(std::vector<Stats> stats) {
		std::vector<double> res;
		res.reserve(stats.size());
		for (auto stat : stats) {
			switch(stats_field_) {
			case STATS_MIN:
				res.push_back(static_cast<double>(stat.min));
				break;
			case STATS_MAX:
				res.push_back(static_cast<double>(stat.max));
				break;
			case STATS_SUM:
				res.push_back(static_cast<double>(stat.sum));
				break;
			case STATS_COUNT:
				res.push_back(static_cast<double>(stat.count));
				break;
			case STATS_AVG:
				res.push_back(stat.average);
				break;
			}
		}
		return res;
	}

    uint width_;
    uint height_;
    uint window_ms_;
    uint num_buckets_;

    StatsField stats_field_ = STATS_SUM;
    RunningAverage stats_;

    TextWidget max_label_;
    TextWidget min_label_;
};

/**
 * Displays text facts for a period of time, stacking them one after another; fading-out opacity.
 * Convenient for warnings, custom messages and pop-ups.
 *
 * @param timeout_ms stop displaying the fact after this many milliseconds since it was received
 */
class PopupWidget: public Widget {
public:
	PopupWidget(int pos_x, int pos_y, uint timeout_ms, uint num_args) :
		Widget(pos_x, pos_y, num_args), timeout_(timeout_ms) {}

	void measure(cairo_t *cr) override {
        auto now = std::chrono::steady_clock::now();
        removeExpired(now);

        double max_width = 0.0;
        double total_height = 0.0;

        for (auto &msg : msgs_) {
            if (!msg.measured) {
                cairo_text_extents_t extents;
                cairo_text_extents(cr, msg.text.c_str(), &extents);
                msg.width = extents.width;
                msg.height = extents.height;
                msg.measured = true;
            }
            max_width = std::max(max_width, msg.width + PADDING * 2);
            total_height += msg.height + PADDING * 2 + ITEM_SPACING;
        }
        setSize(
            static_cast<int>(std::ceil(max_width)),
            static_cast<int>(std::ceil(total_height))
        );
    }

	void draw(cairo_t *cr) override {
		auto [x, y] = xy(cr);
		auto now = std::chrono::steady_clock::now();
		if (removeExpired(now)) {
			invalidateMeasure();
		}
        double y_offset = y;
        for (const auto &msg : msgs_) {
            auto past = std::chrono::duration_cast<std::chrono::milliseconds>(now - msg.time);
            double fade_fraction = 1.0 - static_cast<double>(past.count()) / static_cast<double>(timeout_.count());

            // Draw popup box
            cairo_set_source_rgba(cr, 0.0, 0.0, 0.0, fade_fraction / 3.0);
			cairo_rectangle(cr,
							x - PADDING,
							y_offset + PADDING,
							msg.width + (PADDING * 2), -(msg.height + (PADDING * 2)));
			cairo_fill(cr);

			// Draw popup text
			cairo_set_source_rgba(cr, 255.0, 255.0, 255.0, fade_fraction);
			cairo_move_to(cr, x, y_offset);
			cairo_show_text(cr, msg.text.c_str());
			y_offset += msg.height + (PADDING * 2) + ITEM_SPACING;
		}
	}

    void setFact(uint, Fact fact) override {
        msgs_.push_back({
            .time = std::chrono::steady_clock::now(),
            .text = fact.getStrValue()
        });
        invalidateMeasure();
    }

private:
    struct Message {
        std::chrono::time_point<std::chrono::steady_clock> time;
        std::string text;
        double width = 0.0;
        double height = 0.0;
        bool measured = false;
    };

    bool removeExpired(std::chrono::time_point<std::chrono::steady_clock> now) {
		bool removed = false;
        while (!msgs_.empty() && now - msgs_.front().time > timeout_) {
            msgs_.pop_front();
			removed = true;
        }
		return removed;
    }

    static constexpr double PADDING = 5.0;
    static constexpr double ITEM_SPACING = 2.0;

    std::deque<Message> msgs_;
    std::chrono::milliseconds timeout_;
};

//
// Specific widgets
//

class DvrStatusWidget: public Widget {
public:
    DvrStatusWidget(int pos_x, int pos_y, cairo_surface_t *icon, std::string text):
		Widget(pos_x, pos_y, 1), icon_(0, 0, icon), text_(0, 0, std::move(text), 0, DrawStyle{
			.fill = {1.0, 0.0, 0.0, 1.0},
			.outline = {0.0, 0.0, 0.0, 1.0},
			.outline_width = 2.0
		}) {}

    void measure(cairo_t *cr) override {
        if (!isActive()) {
            setSize(0, 0);
            return;
        }
        measureChild(icon_, cr);
        measureChild(text_, cr);
        setSize(
            icon_.width() + SPACING + text_.width(),
            std::max(icon_.height(), text_.height())
        );
    }

	void draw(cairo_t *cr) override {
        if (!isActive()) {
            return;
		}
        auto [x, y] = xy(cr);
        icon_.drawAt(cr, x, y - 20);
        text_.drawAt(cr, x + icon_.width() + SPACING, y);
    }

private:
    bool isActive() const {
        Fact status = fact(0);
        return status.isDefined() && status.getBoolValue();
    }

	static constexpr int SPACING = 14;

    IconWidget icon_;
    TextWidget text_;
};

class DvrStorageWidget : public Widget {
public:
    DvrStorageWidget(int pos_x, int pos_y, cairo_surface_t *icon):
		Widget(pos_x, pos_y, 2), icon_(0, 0, icon), text_(0, 0, "-") {}

    void measure(cairo_t *cr) override {
        if (!visible_) {
            setSize(0, 0);
            return;
        }
        measureChild(icon_, cr);
        if (!show_text_) {
            setSize(icon_.width(), icon_.height());
            return;
        }
        measureChild(text_, cr);
        setSize(
            icon_.width() + SPACING + text_.width(),
            std::max(icon_.height(), text_.height())
        );
    }

    void draw(cairo_t *cr) override {
        if (!visible_) {
            return;
        }
        auto [x, y] = xy(cr);
        icon_.drawAt(cr, x, y - 20);
		if (show_text_) {
        	text_.drawAt(cr, x + icon_.width() + SPACING, y);
    	}
    }

    void setFact(uint idx, Fact fact) override {
        storeFact(idx, std::move(fact));
        updateState();
    }

private:
    void updateState() {
        Fact status_fact = fact(0);
        if (!status_fact.isDefined()) {
            visible_ = false;
            return;
        }

        const auto status = status_fact.getUintValue();
        switch (status) {
        case 0:
            visible_ = true;
			show_text_ = false;
        	icon_.setFillColor({0.4, 0.4, 0.44, 1.0});
        	icon_.setOutlineColor({0.0, 0.0, 0.0, 0.4});
            break;
        case 1:
            visible_ = true;
			show_text_ = true;
			icon_.setFillColor({1.0, 1.0, 1.0, 1.0});
        	icon_.setOutlineColor({0.0, 0.0, 0.0, 1.0});
            updateStorageText();
            break;
        case 2:
            visible_ = true;
			show_text_ = true;
			icon_.setFillColor({1.0, 0.0, 0.0, 1.0});
        	icon_.setOutlineColor({0.0, 0.0, 0.0, 1.0});
            updateStorageText();
            break;
        default:
            visible_ = false;
            break;
        }
    }

    void updateStorageText() {
        Fact storage_fact = fact(1);
        if (!storage_fact.isDefined()) {
            text_.setText("-");
            return;
        }
        text_.setText(format_storage_size(storage_fact.getUintValue()));
    }

    static std::string format_storage_size(uint64_t bytes) {
        struct StorageUnit {
            uint64_t size;
            const char *name;
        };

        static constexpr StorageUnit units[] = {
            {1ULL << 40, "T"},
            {1ULL << 30, "G"},
            {1ULL << 20, "M"},
            {1ULL << 10, "K"},
        };

        const StorageUnit *unit = &units[3];
        for (const auto &u : units) {
            if (bytes >= u.size) {
                unit = &u;
                break;
            }
        }
        const double value = static_cast<double>(bytes) / unit->size;

        char buf[32];
        snprintf(buf, sizeof(buf), "%.1f %s", value, unit->name);

        return buf;
    }

    static constexpr int SPACING = 5;
    bool visible_ = false;
	bool show_text_ = false;

    IconWidget icon_;
    TextWidget text_;
};

class IconStatusWidget: public IconWidget {
public:
    IconStatusWidget(int pos_x, int pos_y, cairo_surface_t *icon) :
		IconWidget(pos_x, pos_y, icon, 1, DrawStyle{
			.fill = {0.4, 0.4, 0.44, 1.0},
			.outline = {0.0, 0.0, 0.0, 0.4},
			.outline_width = 1.0}) {}

    void setFact(uint idx, Fact fact) override {
        if (fact.isDefined() && fact.getBoolValue()) {
            setFillColor({1.0, 1.0, 1.0, 1.0});
            setOutlineColor({0.0, 0.0, 0.0, 1.0});
        } else {
            setFillColor({0.4, 0.4, 0.44, 1.0});
            setOutlineColor({0.0, 0.0, 0.0, 0.4});
        }
    }
};

class IconTplStatusWidget : public Widget {
public:
    IconTplStatusWidget(int pos_x, int pos_y, cairo_surface_t *icon, std::string tpl, uint num_args) :
		Widget(pos_x, pos_y, num_args), icon_(0, 0, icon), text_(0, 0, std::move(tpl), num_args - 1) {
        setInactiveStyle();
    }

    void measure(cairo_t *cr) override {
        measureChild(icon_, cr);
        measureChild(text_, cr);
        setSize(
            icon_.width() + SPACING + text_.width(),
            std::max(icon_.height(), text_.height())
        );
    }

    void draw(cairo_t *cr) override {
        auto [x, y] = xy(cr);
        icon_.drawAt(cr, x, y - 20);
        text_.drawAt(cr, x + icon_.width() + SPACING, y);
    }

    void setFact(uint idx, Fact fact) override {
        if (idx == 0) {
            if (fact.isDefined() && fact.getBoolValue()) {
                setActiveStyle();
            } else {
                setInactiveStyle();
            }
            return;
        }
        text_.setFact(idx - 1, std::move(fact));
        invalidateMeasure();
    }

private:
    void setActiveStyle() {
        const CairoColor fill{
            1.0, 1.0, 1.0, 1.0
        };
        const CairoColor outline{
            0.0, 0.0, 0.0, 1.0
        };

        icon_.setFillColor(fill);
        icon_.setOutlineColor(outline);
        text_.setFillColor(fill);
        text_.setOutlineColor(outline);
    }

    void setInactiveStyle() {
        const CairoColor fill{
            0.4, 0.4, 0.44, 1.0
        };
        const CairoColor outline{
            0.0, 0.0, 0.0, 0.4
        };

        icon_.setFillColor(fill);
        icon_.setOutlineColor(outline);
        text_.setFillColor(fill);
        text_.setOutlineColor(outline);
    }

    static constexpr int SPACING = 9;

    IconWidget icon_;
    TplTextWidget text_;
};

class VideoWidget: public Widget {
public:
  VideoWidget(int pos_x, int pos_y, uint window_size_ms, uint bucket_size_ms,
              cairo_surface_t *icon, std::string tpl, uint refresh_rate, uint num_args) :
		Widget(pos_x, pos_y, num_args), icon_(0, 0, icon), text_(0, 0, std::move(tpl), num_args),
        fps_(window_size_ms, bucket_size_ms) 
    {
        if (refresh_rate < refresh_frequency_ms || refresh_rate > MAX_WIDGET_REFRESH_MS) {
            spdlog::warn("VideoWidget: Refresh rate '{}' is out of range [{} {}].",
                refresh_rate, refresh_frequency_ms, MAX_WIDGET_REFRESH_MS);
            spdlog::warn("VideoWidget: Using osd refresh rate: {}", refresh_frequency_ms);
            refresh_rate_ms_ = std::chrono::milliseconds(refresh_frequency_ms);
        }
        else {
            refresh_rate_ms_ = std::chrono::milliseconds(refresh_rate);
        }
    }

    void measure(cairo_t* cr) override {
        measureChild(icon_, cr);
        measureChild(text_, cr);
        setSize(
            icon_.width() + SPACING + text_.width(),
            std::max(icon_.height(), text_.height())
        );
    }

    void draw(cairo_t* cr) override {
        auto [x, y] = xy(cr);
        icon_.drawAt(cr, x, y - 20);
        text_.drawAt(cr, x + icon_.width() + SPACING, y);
    }

    void setFact(uint idx, Fact fact) override {
        if (idx != 0) {
            text_.setFact(idx, std::move(fact));
            invalidateMeasure();
            return;
        }
        if (!fact.isDefined()) {
            text_.setFact(idx, Fact());
            invalidateMeasure();
            return;
        }
		// replace the value with its increment rate per-second
        ulong num_frames = fact.getUintValue(); // should be always '1'
        fps_.add(num_frames);

        auto now = std::chrono::steady_clock::now();
        auto elapsed = now - last_drawn_;

        if (elapsed < refresh_rate_ms_) {
            return;
        }
        last_drawn_ = now;
        text_.setFact(idx, Fact(FactMeta("video_fps"), (ulong)fps_.rate_per_second_over_last_ms(1000)));
        invalidateMeasure();
    }

private:
    static constexpr int SPACING = 14;

    IconWidget icon_;
    TplTextWidget text_;

    RunningAverage fps_;
    std::chrono::milliseconds refresh_rate_ms_{};
    std::chrono::steady_clock::time_point last_drawn_{};
};

class VideoBitrateWidget: public Widget {
public:
  VideoBitrateWidget(int pos_x, int pos_y, uint window_size_ms, uint bucket_size_ms,
                     cairo_surface_t *icon, std::string tpl, uint refresh_rate, uint num_args) :
        Widget(pos_x, pos_y, num_args), icon_(0, 0, icon), text_(0, 0, std::move(tpl), num_args),
        bps_(window_size_ms, bucket_size_ms)
    {
        assert(num_args == 1);
        if (refresh_rate < refresh_frequency_ms || refresh_rate > MAX_WIDGET_REFRESH_MS) {
            spdlog::warn("VideoBitrateWidget: Refresh rate '{}' is out of range [{} {}].",
                refresh_rate, refresh_frequency_ms, MAX_WIDGET_REFRESH_MS);
            spdlog::warn("VideoBitrateWidget: Using osd refresh rate: {}", refresh_frequency_ms);
            refresh_rate_ms_ = std::chrono::milliseconds(refresh_frequency_ms);
        }
        else {
            refresh_rate_ms_ = std::chrono::milliseconds(refresh_rate);
        }
    }

    void measure(cairo_t* cr) override {
        measureChild(icon_, cr);
        measureChild(text_, cr);
        setSize(
            icon_.width() + SPACING + text_.width(),
            std::max(icon_.height(), text_.height())
        );
    }

    void draw(cairo_t* cr) override {
        auto [x, y] = xy(cr);
        icon_.drawAt(cr, x, y - 20);
        text_.drawAt(cr, x + icon_.width() + SPACING, y);
    }

    void setFact(uint idx, Fact fact) override {
        assert(idx == 0);
        if (!fact.isDefined()) {
            text_.setFact(idx, Fact());
            invalidateMeasure();
            return;
        }
		// replace the value with its increment rate per-second
		ulong num_bytes = fact.getUintValue();
        bps_.add(num_bytes);

        auto now = std::chrono::steady_clock::now();
        auto elapsed = now - last_drawn_;
            
        if (elapsed < refresh_rate_ms_) {
            return;
        }
        last_drawn_ = now;

        // 125000 is 1_000_000 / 8 (megabits, not megabytes)
        text_.setFact(idx, Fact(FactMeta("video_mbps"), bps_.rate_per_second_over_last_ms(1000) / 125000.0));
        invalidateMeasure();
    }

private:
    static constexpr int SPACING = 14;

    IconWidget icon_;
    TplTextWidget text_;

    RunningAverage bps_;
    std::chrono::milliseconds refresh_rate_ms_{};
    std::chrono::steady_clock::time_point last_drawn_{};
};

class VideoDecodeLatencyWidget: public Widget {
public:
  VideoDecodeLatencyWidget(int pos_x, int pos_y, uint window_size_ms, uint bucket_size_ms,
                           cairo_surface_t *icon, std::string tpl, uint refresh_rate, uint num_args) :
        Widget(pos_x, pos_y, num_args), icon_(0, 0, icon), text_(0, 0, std::move(tpl), 3),
        timing_(window_size_ms, bucket_size_ms)
    {
        assert(num_args == 1);
        if (refresh_rate < refresh_frequency_ms || refresh_rate > MAX_WIDGET_REFRESH_MS) {
            spdlog::warn("VideoDecodeLatencyWidget: Refresh rate '{}' is out of range [{} {}].",
                refresh_rate, refresh_frequency_ms, MAX_WIDGET_REFRESH_MS);
            spdlog::warn("VideoDecodeLatencyWidget: Using osd refresh rate: {}", refresh_frequency_ms);
            refresh_rate_ms_ = std::chrono::milliseconds(refresh_frequency_ms);
        }
        else {
            refresh_rate_ms_ = std::chrono::milliseconds(refresh_rate);
        }
    }

    void measure(cairo_t *cr) override {
        measureChild(icon_, cr);
        measureChild(text_, cr);
        setSize(
            icon_.width() + SPACING + text_.width(),
            std::max(icon_.height(), text_.height())
        );
    }

    void draw(cairo_t *cr) override {
        auto [x, y] = xy(cr);
        icon_.drawAt(cr, x, y - 20);
        text_.drawAt(cr, x + icon_.width() + SPACING, y);
    }

    void setFact(uint idx, Fact fact) override {
        assert(idx == 0);
        if (!fact.isDefined()) {
            text_.setFact(0, Fact());
            text_.setFact(1, Fact());
            text_.setFact(2, Fact());
            invalidateMeasure();
            return;
        }
        const ulong decode_time = fact.getUintValue();
        timing_.add(decode_time);

        auto now = std::chrono::steady_clock::now();
        auto elapsed = now - last_drawn_;
            
        if (elapsed < refresh_rate_ms_) {
            return;
        }
        last_drawn_ = now;

        Stats stats = timing_.get_stats_over_last_ms_result(1000);
        text_.setFact(0, Fact(FactMeta("video_avg"), stats.average));
        text_.setFact(1, Fact(FactMeta("video_min"), stats.min));
        text_.setFact(2, Fact(FactMeta("video_max"), stats.max));
        invalidateMeasure();
    }

private:
    static constexpr int SPACING = 14;

    IconWidget icon_;
    TplTextWidget text_;

    RunningAverage timing_;
    std::chrono::milliseconds refresh_rate_ms_{};
    std::chrono::steady_clock::time_point last_drawn_{};
};

class GPSWidget: public TextWidget {
public:
	GPSWidget(int pos_x, int pos_y, uint num_args) :
		TextWidget(pos_x, pos_y, "", num_args) {
		assert(num_args == 3);
	}

    void measure(cairo_t *cr) override {
        if (!isReady()) {
            setSize(0, 0);
            return;
        }
        setText(formatText());
        TextWidget::measure(cr);
    }

    void draw(cairo_t *cr) override {
        if (!isReady())
            return;
        auto [x, y] = xy(cr);
        drawAt(cr, x + TEXT_OFFSET_X, y);
    }

private:
    bool isReady() const {
        Fact fix = fact(0);
        Fact lat = fact(1);
        Fact lon = fact(2);
        return fix.isDefined() && lat.isDefined() && lon.isDefined();
    }

    std::string formatText() const {
        Fact fix_fact = fact(0);
        Fact lat_fact = fact(1);
        Fact lon_fact = fact(2);
        std::string fix_type = "undef";
		char buf[64];
		switch (fix_fact.getUintValue()) {
		case 0:
			fix_type = "no GPS";
			break;
		case 1:
			fix_type = "no fix";
			break;
		case 2:
			fix_type = "2D fix";
			break;
		case 3:
			fix_type = "3D fix";
			break;
		case 4:
			fix_type = "DGPS/SBAS 3D";
			break;
		case 5:
			fix_type = "RTK float 3D";
			break;
		case 6:
			fix_type = "RTK Fixed 3D";
			break;
		case 7:
			fix_type = "Static fixed";
			break;
		case 8:
			fix_type = "PPP 3D";
			break;
		}
        const double lat = lat_fact.getIntValue() * 1.0e-7;
        const double lon = lon_fact.getIntValue() * 1.0e-7;
        std::snprintf(buf, sizeof(buf), "%s Lat:%f, Lon:%f", fix_type.c_str(), lat, lon);
        return buf;
    }

    static constexpr int TEXT_OFFSET_X = 40;
};

class DebugWidget: public Widget {
public:
    DebugWidget(int pos_x, int pos_y, uint num_args) :
		Widget(pos_x, pos_y, num_args), lines_(num_args, "undef") {}

    void measure(cairo_t *cr) override {
        double max_width = 0.0;
        for (const auto &line : lines_) {
            cairo_text_extents_t extents;
            cairo_text_extents(cr, line.c_str(), &extents);
            max_width = std::max(max_width, extents.width);
        }
        setSize(
            static_cast<int>(std::ceil(max_width)),
            static_cast<int>(lines_.size() * LINE_HEIGHT)
        );
    }

    void draw(cairo_t *cr) override {
        auto [x, y] = xy(cr);
        auto y_offset = y;
        for (const auto &line : lines_) {
            cairo_set_source_rgba(cr, 255.0, 50.0, 50.0, 1.0);
            cairo_move_to(cr, x, y_offset);
            cairo_show_text(cr, line.c_str());
            y_offset += LINE_HEIGHT;
            SPDLOG_INFO("dbg draw {}", line);
        }
    }

    void setFact(uint idx, Fact fact) override {
        lines_.at(idx) = formatFact(std::move(fact));
        invalidateMeasure();
    }

private:
    static std::string formatFact(Fact fact) {
        std::ostringstream oss;

        if (!fact.isDefined()) {
            oss << "undef";
        } else {
            oss << fact.getName() << " (" << fact.getTypeName() << ") {";
            for (const auto &tag : fact.getTags()) {
                oss << tag.first << "=>" << tag.second << ", ";
            }
            oss << "} = " << fact.asString();
        }
        return oss.str();
    }

    static constexpr int LINE_HEIGHT = 20;

    std::vector<std::string> lines_;
};

class ExternalSurfaceWidget: public Widget {
public:
	ExternalSurfaceWidget(int pos_x, int pos_y, std::string shm_name ): Widget(pos_x, pos_y), shm_name(shm_name)  {};

	void measure(cairo_t *cr) override {
        cairo_surface_t *target = cairo_get_target(cr);
        const int width = cairo_image_surface_get_width(target);
        const int height = cairo_image_surface_get_height(target);
        setSize(width, height);
    }

	void init_shm(cairo_t *cr) {
		SPDLOG_INFO("Creating shm region {}", shm_name);

		cairo_surface_t *target = cairo_get_target(cr);
		int width = cairo_image_surface_get_width(target);
		int height = cairo_image_surface_get_height(target);

        const uint32_t stride   = static_cast<uint32_t>(width * 4); // ARGB32
		const size_t   buf_size = static_cast<size_t>(stride) * height;

		// Calculate total shared memory size
		shm_size = sizeof(SharedMemoryRegion) + (buf_size * SHM_BUFFERS_COUNT); // Metadata + 3 buffers for Image data

		// Create shared memory region
		int shm_fd = shm_open(shm_name.c_str(), O_CREAT | O_RDWR, 0666);
		if (shm_fd == -1) {
			perror("Failed to create shared memory");
			return;
		}

		if (ftruncate(shm_fd, shm_size) == -1) {
			perror("Failed to set shared memory size");
			shm_unlink(shm_name.c_str());
            close(shm_fd);
			return;
		}

		// Map shared memory to process address space
		shm_region = static_cast<SharedMemoryRegion*>(
			mmap(0, shm_size, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0)
		);
		if (shm_region == MAP_FAILED) {
			perror("Failed to map shared memory");
			shm_unlink(shm_name.c_str());
            close(shm_fd);
			return;
		}

        close(shm_fd);

		// Write metadata
		shm_region->width = width;
		shm_region->height = height;
        shm_region->stride = stride;
        shm_region->refresh_rate = refresh_frequency_ms;
        shm_region->ready_index.store(-1);
        shm_region->front_index.store(0);
        shm_region->back_index.store(1);

        unsigned char *base = shm_region->data;

        for (int i = 0; i < SHM_BUFFERS_COUNT; ++i) {
            unsigned char *buf_ptr = base + (i * buf_size);

            // Create Cairo surface for the image data
            cairo_surface_t *surf = cairo_image_surface_create_for_data(
                buf_ptr, CAIRO_FORMAT_ARGB32, width, height, stride);

            if (cairo_surface_status(surf) != CAIRO_STATUS_SUCCESS) {
                spdlog::error("Failed to create cairo surface for buffer {}", i);
                cairo_surface_destroy(surf);
                shm_surfaces[i] = nullptr;
            } else {
                shm_surfaces[i] = surf;
            }

        }
		// Store pointer for cleanup
		shm_data = reinterpret_cast<unsigned char*>(shm_region);
	}

	void draw(cairo_t *cr) override {
        if (!shm_region) {
            init_shm(cr);
        }
        if (!shm_region) {
            return;
        }

		int ready = shm_region->ready_index.exchange(-1);
		if (ready >= 0 && ready < SHM_BUFFERS_COUNT) {
			last_surface_index = ready;
			shm_region->front_index.store(ready);
        }
		if (last_surface_index != -1) {
			cairo_surface_mark_dirty(shm_surfaces[last_surface_index]);
			auto [x, y] = xy(cr);
        	cairo_set_source_surface(cr, shm_surfaces[last_surface_index], x, y); // Position at (0, 0)
        	cairo_paint(cr); // Paint shm_surface onto base_surface
		} 
    }

    ~ExternalSurfaceWidget() override {
		SPDLOG_INFO("Destroying shm region {}", shm_name);

        for (int i = 0; i < SHM_BUFFERS_COUNT; ++i) {
            if (shm_surfaces[i]) {
                cairo_surface_destroy(shm_surfaces[i]);
                shm_surfaces[i] = nullptr;
            }
        }
		if (shm_data) {
			munmap(shm_data, shm_size);
		}
		shm_unlink(shm_name.c_str());
	}

protected:
	SharedMemoryRegion *shm_region = nullptr;
	int32_t last_surface_index = -1;
	cairo_surface_t *shm_surfaces[SHM_BUFFERS_COUNT] = {nullptr, nullptr, nullptr};
	size_t shm_size = 0;
    unsigned char *shm_data = nullptr;
	std::string shm_name;
};

class IconSelectorWidget : public Widget {
public:
    IconSelectorWidget(int pos_x, int pos_y, const std::vector<std::pair<std::pair<int, int>, std::filesystem::path>>& ranges_and_icons, const std::filesystem::path& assets_dir)
        : Widget(pos_x, pos_y, 1), assets_dir_(assets_dir) {
        // Load and cache all icons during initialization
        for (const auto& [range, icon_path] : ranges_and_icons) {
            cairo_surface_t* icon = openIcon(icon_path);
            if (icon) {
                icon_cache_[range] = icon;
            }
        }
    }

    ~IconSelectorWidget() override {
        // Clean up cached icons
        for (auto& [range, icon] : icon_cache_) {
            if (icon) {
                cairo_surface_destroy(icon);
            }
        }
    }

	void measure(cairo_t *) override {
        if (!current_icon_) {
            setSize(0, 0);
            return;
        }
        int width = cairo_image_surface_get_width(current_icon_);
        int height = cairo_image_surface_get_height(current_icon_);
        setSize(width + 2, height + 2);
    }

    void draw(cairo_t *cr) override {
        if (!current_icon_) {
			return;
		}
        auto [x, y] = xy(cr);
		cairo_save(cr);
        cairo_set_source_rgba(cr, 0.0, 0.0, 0.0, 1.0);
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                if (dx * dx + dy * dy > 1 * 1)
                    continue;
                cairo_mask_surface(cr, current_icon_, x + dx, y + dy);
            }
        }
        cairo_set_source_rgba(cr, 1.0, 1.0, 1.0, 1.0);
        cairo_mask_surface(cr, current_icon_, x, y);
        cairo_restore(cr);
    }

    void setFact(uint idx, Fact fact) override {
        assert(idx == 0);
        cairo_surface_t *icon = selectIcon(fact);
        if (icon == current_icon_) {
            return;
        }
        current_icon_ = icon;
        invalidateMeasure();
    }

private:
    cairo_surface_t* selectIcon(Fact& fact) {
        if (!fact.isDefined()) return nullptr;

        long value = 0;
        
        // Convert all fact types to comparable integer values
        switch (fact.getType()) {
            case Fact::T_BOOL:
                value = fact.getBoolValue() ? 1 : 0;
                break;
            case Fact::T_INT:
                value = fact.getIntValue();
                break;
            case Fact::T_UINT:
                value = static_cast<long>(fact.getUintValue());
                break;
            case Fact::T_DOUBLE:
                value = static_cast<long>(fact.getDoubleValue());
                break;
            case Fact::T_STRING:
                try {
                    value = std::stol(fact.getStrValue());
                } catch (...) {
                    // If string can't be converted to number, use 0
                    value = 0;
                }
                break;
            case Fact::T_UNDEF:
            default:
                return nullptr;
        }

        // Iterate through the configured ranges and select the appropriate icon
        for (const auto& [range, icon] : icon_cache_) {
            if (value >= range.first && value <= range.second) {
                return icon;
            }
        }

        return nullptr; // No icon selected
    }

    cairo_surface_t* openIcon(const std::filesystem::path& icon_path) {
        std::filesystem::path full_path = assets_dir_ / icon_path;
        cairo_surface_t* icon = cairo_image_surface_create_from_png(full_path.c_str());
        if (cairo_surface_status(icon) != CAIRO_STATUS_SUCCESS) {
            spdlog::error("Failed to open icon: {}", full_path.string());
			cairo_surface_destroy(icon);
            return nullptr;
        }
        return icon;
    }

    std::map<std::pair<int, int>, cairo_surface_t*> icon_cache_; // Cache of loaded icons
    std::filesystem::path assets_dir_;
    cairo_surface_t* current_icon_ = nullptr; // Currently selected icon
};

//
// Layouts
//

class Layout {
public:
    Layout(int pos_x, int pos_y, int spacing) : pos_x_(pos_x), pos_y_(pos_y), spacing_(spacing) {}

    virtual ~Layout() = default;

    Layout(const Layout&) = delete;
    Layout& operator=(const Layout&) = delete;

    void addWidget(Widget *widget) {
        widgets_.push_back(widget);
        dirty_ = true;
    }

    void invalidate() {
        dirty_ = true;
    }

    bool dirty() const {
        return dirty_;
    }

    void update(cairo_t *cr) {
        if (!dirty_) {
            return;
        }
        doUpdate(cr);
        dirty_ = false;
    }

protected:
    virtual void doUpdate(cairo_t *cr) = 0;

    int x(cairo_t *cr) const {
        cairo_surface_t *target = cairo_get_target(cr);
        int width = cairo_image_surface_get_width(target);
        return (width + pos_x_) % width;
    }

    int y(cairo_t *cr) const {
        cairo_surface_t *target = cairo_get_target(cr);
        int height = cairo_image_surface_get_height(target);
        return (height + pos_y_) % height;
    }

    const std::vector<Widget*>& widgets() const {
        return widgets_;
    }

    int spacing() const {
        return spacing_;
    }

private:
    int pos_x_;
    int pos_y_;
    int spacing_;

    bool dirty_ = true;
    std::vector<Widget*> widgets_;
};

class HorizontalLayout : public Layout {
public:
    enum class Direction {
        LeftToRight,
        RightToLeft
    };

    HorizontalLayout(int pos_x, int pos_y, int spacing, Direction direction) : 
		Layout(pos_x, pos_y, spacing), direction_(direction) {}

protected:
	void doUpdate(cairo_t *cr) override {
        int cursor_x = x(cr);
        const int pos_y = y(cr);
        const bool rtl = direction_ == Direction::RightToLeft;
        bool first = true;
        for (Widget *widget : widgets()) {
            if (widget->width() == 0 && widget->height() == 0) {
                continue;
            }
            if (!first) {
                cursor_x += rtl ? -spacing() : spacing();
            }
            if (rtl) {
                cursor_x -= widget->width();
            }
            widget->setPosition(cursor_x, pos_y);
            if (!rtl) {
                cursor_x += widget->width();
            }
            first = false;
        }
    }

private:
    Direction direction_;
};

class Osd {
public:
    ~Osd() {
        if (screensaver_image) {
            cairo_surface_destroy(screensaver_image);
        }
    }

	void loadConfig(json cfg) {
		json obj;
		if (!cfg.contains("format")) {
			spdlog::error("OSD config doesn't have 'format' key");
			return;
		}
		if (!cfg.contains("widgets")) {
			//|| cfg["widgets"].type() != json::value_t::array)
			spdlog::error("OSD config doesn't have 'widgets' key");
			return;
		}
        std::filesystem::path assets_dir{"."};
		if (cfg.contains("assets_dir")) {
			assets_dir = cfg.at("assets_dir").template get<std::filesystem::path>();
		}
		json widgets_j = cfg.at("widgets");
		for (json widget_j : widgets_j) {
			if(!widget_j.contains("name") || !widget_j.contains("type") || !widget_j.contains("x") ||
				!widget_j.contains("y") || !widget_j.contains("facts")) {
				spdlog::error("Missing required key name/type/x/y/facts");
				return;
			}
			auto name = widget_j.at("name").template get<std::string>();
			auto type = widget_j.at("type").template get<std::string>();
			auto x = widget_j.at("x").template get<int>();
			auto y = widget_j.at("y").template get<int>();
			std::vector<FactMatcher> matchers;
			for(json matcher_j : widget_j.at("facts")) {
				auto matcher_name = matcher_j.at("name").template get<std::string>();
				FactTags tags;
				if (matcher_j.contains("tags")) {
					for (auto& [key, value] : matcher_j.at("tags").items()) {
						tags.insert({key, value});
					}
				}
				matchers.push_back(FactMatcher(matcher_name, tags));
			}
			if (type == "TextWidget") {
				addWidget(std::make_unique<TextWidget>(x, y, widget_j.at("text").template get<std::string>()),
						  matchers);
			}
			else if (type == "ExternalSurfaceWidget") {
				addWidget(std::make_unique<ExternalSurfaceWidget>(x, y, name), matchers);
			} else if (type == "IconSelectorWidget") {
				std::vector<std::pair<std::pair<int, int>, std::filesystem::path>> ranges_and_icons;
				for (const auto& range_icon : widget_j.at("ranges_and_icons")) {
					int range_start = range_icon.at("range")[0];
					int range_end = range_icon.at("range")[1];
					std::filesystem::path icon_path = range_icon.at("icon_path");
					ranges_and_icons.push_back({{range_start, range_end}, icon_path});
				}
				addWidget(std::make_unique<IconSelectorWidget>(x, y, ranges_and_icons, assets_dir), matchers);
			} else if (type == "TplTextWidget") {
				auto tpl = widget_j.at("template").template get<std::string>();
				addWidget(std::make_unique<TplTextWidget>(x, y, tpl, (uint)matchers.size()), matchers);
			} else if(type == "IconTplTextWidget") {
				auto tpl = widget_j.at("template").template get<std::string>();
				auto icon_path = widget_j.at("icon_path").template get<std::filesystem::path>();
				cairo_surface_t *icon = openIcon(name, assets_dir, icon_path);
				if (icon == NULL) break;
				addWidget(std::make_unique<IconTplTextWidget>(x, y, icon, tpl, (uint)matchers.size()), matchers);
			} else if(type == "DvrStatusWidget") {
				auto id = widget_j.at("id").get<std::string>();
				auto text = widget_j.at("text").template get<std::string>();
				auto icon_path = widget_j.at("icon_path").template get<std::filesystem::path>();
				cairo_surface_t *icon = openIcon(name, assets_dir, icon_path);
				if (icon == NULL) break;
				addWidget(std::make_unique<DvrStatusWidget>(x, y, icon, text), matchers, id);
			} else if (type == "DvrStorageWidget") {
				auto id = widget_j.at("id").get<std::string>();
				auto icon_path = widget_j.at("icon_path").template get<std::filesystem::path>();
				cairo_surface_t *icon = openIcon(name, assets_dir, icon_path);
				if (icon == NULL) break;		
				addWidget(std::make_unique<DvrStorageWidget>(x, y, icon), matchers, id);
			} else if(type == "IconStatusWidget") {
				auto icon_path = widget_j.at("icon_path").template get<std::filesystem::path>();
				cairo_surface_t *icon = openIcon(name, assets_dir, icon_path);
				if (icon == NULL) break;
				addWidget(std::make_unique<IconStatusWidget>(x, y, icon), matchers);
			} else if (type == "IconTplStatusWidget") {
    			auto tpl = widget_j.at("template").template get<std::string>();
    			auto icon_path = widget_j.at("icon_path").template get<std::filesystem::path>();
    			cairo_surface_t *icon = openIcon(name, assets_dir, icon_path);
    			if (icon == NULL) break;
    			addWidget(std::make_unique<IconTplStatusWidget>(x, y, icon, tpl, (uint)matchers.size()), matchers);
			} else if(type == "VideoWidget") {
				auto tpl = widget_j.at("template").template get<std::string>();
				auto icon_path = widget_j.at("icon_path").template get<std::filesystem::path>();
				uint window_size_s = widget_j.at("per_second_window_s").template get<uint>();
                uint bucket_size_ms = widget_j.at("per_second_bucket_ms").template get<uint>();
                uint refresh_rate_ms = widget_j.value("refresh_rate_ms", refresh_frequency_ms);
				cairo_surface_t *icon = openIcon(name, assets_dir, icon_path);
				if (icon == NULL) break;
				addWidget(std::make_unique<VideoWidget>(x, y, window_size_s * 1000, bucket_size_ms,
                                          icon, tpl, refresh_rate_ms, (uint)matchers.size()),
						  matchers);
			} else if(type == "VideoBitrateWidget") {
				auto tpl = widget_j.at("template").template get<std::string>();
				auto icon_path = widget_j.at("icon_path").template get<std::filesystem::path>();
				uint window_size_s = widget_j.at("per_second_window_s").template get<uint>();
                uint bucket_size_ms = widget_j.at("per_second_bucket_ms").template get<uint>();
                uint refresh_rate_ms = widget_j.value("refresh_rate_ms", refresh_frequency_ms);
				cairo_surface_t *icon = openIcon(name, assets_dir, icon_path);
				if (icon == NULL) break;
				addWidget(std::make_unique<VideoBitrateWidget>(x, y, window_size_s * 1000, bucket_size_ms,
                                                 icon, tpl, refresh_rate_ms, (uint)matchers.size()),
						  matchers);
			} else if(type == "VideoDecodeLatencyWidget") {
				auto tpl = widget_j.at("template").template get<std::string>();
				auto icon_path = widget_j.at("icon_path").template get<std::filesystem::path>();
				uint window_size_s = widget_j.at("per_second_window_s").template get<uint>();
                uint bucket_size_ms = widget_j.at("per_second_bucket_ms").template get<uint>();
                uint refresh_rate_ms = widget_j.value("refresh_rate_ms", refresh_frequency_ms);
				cairo_surface_t *icon = openIcon(name, assets_dir, icon_path);
				if (icon == NULL) break;
				addWidget(std::make_unique<VideoDecodeLatencyWidget>(x, y, window_size_s * 1000, bucket_size_ms,
                                                       icon, tpl, refresh_rate_ms, 1),
						  matchers);
			} else if(type == "BoxWidget") {
				auto width = widget_j.at("width").template get<uint>();
				auto height = widget_j.at("height").template get<uint>();
				json color_j = widget_j.at("color");
				auto r = color_j.at("r").template get<double>();
				auto g = color_j.at("g").template get<double>();
				auto b = color_j.at("b").template get<double>();
				auto a = color_j.at("alpha").template get<double>();
				addWidget(std::make_unique<BoxWidget>(x, y, width, height, CairoColor{r, g, b, a}), matchers);
			} else if(type == "BarChartWidget") {
				auto width = widget_j.at("width").template get<uint>();
				auto height = widget_j.at("height").template get<uint>();
				auto window_s = widget_j.at("window_s").template get<uint>();
				auto num_buckets = widget_j.at("num_buckets").template get<uint>();
				auto stats_kind_str = widget_j.at("stats_kind").template get<std::string>();
				BarChartWidget::StatsField stats_kind;
				if (stats_kind_str == "sum") {
					stats_kind = BarChartWidget::STATS_SUM;
				} else if (stats_kind_str == "min") {
					stats_kind = BarChartWidget::STATS_MIN;
				} else if (stats_kind_str == "max") {
					stats_kind = BarChartWidget::STATS_MAX;
				} else if (stats_kind_str == "count") {
					stats_kind = BarChartWidget::STATS_COUNT;
				} else if (stats_kind_str == "avg") {
					stats_kind = BarChartWidget::STATS_AVG;
				} else {
					SPDLOG_WARN("{}: invalid stats_kind {}", name, stats_kind_str);
					break;
				}
				addWidget(std::make_unique<BarChartWidget>(x, y, width, height, window_s, num_buckets, stats_kind),
						  matchers);
			} else if (type == "GPSWidget") {
				addWidget(std::make_unique<GPSWidget>(x, y, (uint)matchers.size()), matchers);
			} else if(type == "PopupWidget") {
				auto timeout_ms = widget_j.at("timeout_ms").template get<uint>();
				addWidget(std::make_unique<PopupWidget>(x, y, timeout_ms, (uint)matchers.size()),
						  matchers);
			} else if(type == "DebugWidget") {
				addWidget(std::make_unique<DebugWidget>(x, y, (uint)matchers.size()), matchers);
			} else {
				spdlog::warn("Widget '{}': unknown type: {}", name, type);
			}
		}
		if (cfg.contains("layouts")) {
			json layouts_j = cfg.at("layouts");
			for (json layout_json : layouts_j) {
				if (!layout_json.contains("type") || !layout_json.contains("x") || 
					!layout_json.contains("y") || !layout_json.contains("spacing")) {
					spdlog::error("Invalid layout configuration");
					return;
				}
				const auto type = layout_json.at("type").get<std::string>();
				const auto x = layout_json.at("x").get<int>();
				const auto y = layout_json.at("y").get<int>();
				const auto spacing = layout_json.at("spacing").get<int>();

				std::unique_ptr<Layout> layout;
				if (type == "horizontal") {
					if (!layout_json.contains("direction")) {
						spdlog::error("Horizontal layout has no direction");
						return;
					}
					const auto direction = layout_json.at("direction").get<std::string>();
					HorizontalLayout::Direction dir;
					if (direction == "left-to-right") {
						dir = HorizontalLayout::Direction::LeftToRight;
					} else if (direction == "right-to-left") {
						dir = HorizontalLayout::Direction::RightToLeft;
					} else {
						spdlog::error("Unknown horizontal layout direction '{}'", direction);
						continue;
					}
					layout = std::make_unique<HorizontalLayout>(x, y, spacing, dir);
				} else {
					spdlog::error("Unknown layout type '{}'", type);
					continue;
				}
				if (!layout_json.contains("widgets")) {
					spdlog::error("Layout has no widgets");
					return;
				}
				Layout *layout_ptr = layout.get();
				json widget_ids_j = layout_json.at("widgets");
				for (json widget_id_json : widget_ids_j) {
					const auto widget_id = widget_id_json.get<std::string>();
					auto it = widgets_by_id.find(widget_id);
					if (it == widgets_by_id.end()) {
						spdlog::error("Layout references unknown widget '{}'", widget_id);
						continue;
					}
					addWidgetToLayout(layout_ptr, it->second);
				}
				layouts.push_back(std::move(layout));
			}
		}
	}

	Osd *addWidget(std::unique_ptr<Widget> widget, std::vector<FactMatcher> param_matchers, const std::string& id = "") {
		uint arg_idx = 0;
		Widget *widget_ptr = widget.get();
		widgets.push_back(std::move(widget));
		for (auto matcher : param_matchers) {
			matchers.push_back(std::make_tuple(matcher, widget_ptr, arg_idx));
			arg_idx++;
		}
		if (!id.empty()) {
			if (widgets_by_id.find(id) != widgets_by_id.end()) {
				spdlog::error("Duplicate widget id '{}'", id);
			} else {
				widgets_by_id[id] = widget_ptr;
			}
		}
		return this;
	};

	void addWidgetToLayout(Layout *layout, Widget *widget) {
		layout->addWidget(widget);
		widget_layouts[widget].push_back(layout);
	}

	void measureWidgets(cairo_t* cr) {
		for (auto& widget : widgets) {
			if (!widget->measureDirty()) {
				continue;
			}
			const int old_width = widget->width();
			const int old_height = widget->height();

			widget->measure(cr);
			if (old_width == widget->width() && old_height == widget->height()) {
				continue;
			}
			auto it = widget_layouts.find(widget.get());
			if (it == widget_layouts.end()) {
				continue;
			}
			for (Layout *layout : it->second) {
				layout->invalidate();
			}
		}
	}

	void draw(cairo_t *cr) {
		measureWidgets(cr);
		for (auto &layout : layouts) {
			layout->update(cr);
		}
		for(auto &widget : widgets)
			widget->draw(cr);
	};

	void setFact(Fact fact) {
		for (auto [matcher, widget, arg_idx] : matchers) {
			if (fact.matches(matcher)) {
				widget->setFact(arg_idx, fact);
			}
		}
	};

    void loadScreensaverImage(std::string image_path)  {

        cairo_surface_t *image = cairo_image_surface_create_from_png(image_path.c_str());
        if (cairo_surface_status(image) != CAIRO_STATUS_SUCCESS) {
            spdlog::error("Can't open icon '{}' for screensaver", image_path);
            return;
        }
        screensaver_image = image;
    }

    cairo_surface_t * getScreensaverImage() const {
        return screensaver_image;
    }

private:

	cairo_surface_t *openIcon(std::string widget_name, std::filesystem::path base_path,
							  std::filesystem::path icon_path) {
		if (icon_path.is_relative()) {
			icon_path = base_path / icon_path;
		}
		cairo_surface_t *icon = cairo_image_surface_create_from_png(icon_path.c_str());
		if (cairo_surface_status(icon) != CAIRO_STATUS_SUCCESS) {
			std::string status("OTHER_ERROR");
			switch (cairo_surface_status(icon)) {
			case CAIRO_STATUS_NULL_POINTER:
				status = "NULL_POINTER";
				break;
			case CAIRO_STATUS_NO_MEMORY:
				status = "NO_MEMORY";
				break;
			case CAIRO_STATUS_READ_ERROR:
				status = "READ_ERROR";
				break;
			case CAIRO_STATUS_INVALID_CONTENT:
				status = "INVALID_CONTENT";
				break;
			case CAIRO_STATUS_INVALID_FORMAT:
				status = "INVALID_FORMAT";
				break;
			case CAIRO_STATUS_INVALID_VISUAL:
				status = "INVALID_VISUAL";
				break;
			};
			spdlog::error("Widget '{}': Can't open icon '{}': {}",
						  widget_name, icon_path.string(), status);
			return NULL;
		}
		return icon;
	}

	
	std::vector<std::unique_ptr<Layout>> layouts;
	std::unordered_map<Widget*, std::vector<Layout*>> widget_layouts;
	std::unordered_map<std::string, Widget*> widgets_by_id;

	std::vector<std::unique_ptr<Widget>> widgets;
	std::vector<std::tuple<FactMatcher, Widget *, uint>> matchers;
    cairo_surface_t * screensaver_image = nullptr;
};

void show_screensaver(cairo_t* cr, int width, int height, cairo_surface_t * screensaver_image) {
    // draw background
	cairo_set_operator(cr, CAIRO_OPERATOR_SOURCE);
    cairo_set_source_rgba(cr, 0.0, 0.0, 0.0, 1.0); // black color
    cairo_rectangle(cr, 0, 0, width, height);
    cairo_fill(cr);

    if (screensaver_image) {
        int image_width = cairo_image_surface_get_width(screensaver_image);
        int image_height = cairo_image_surface_get_height(screensaver_image);

        if (image_width > width || image_height > height) {
			spdlog::error("Icon {} x {} larger than screen {} x {}",
              			  image_width, image_height, width, height);
            return;
        }
        int x = (width - image_width) / 2;
        int y = (height - image_height) / 2;

        cairo_set_source_surface(cr, screensaver_image, x, y);
        cairo_rectangle(cr, x, y, image_width, image_height);
        cairo_fill(cr);
    }
	cairo_set_operator(cr, CAIRO_OPERATOR_OVER);
}

std::queue<Fact> fact_queue;
std::mutex mtx;
std::condition_variable cv;
pthread_mutex_t osd_mutex;

void modeset_paint_buffer(struct modeset_buf *buf, Osd *osd) {
	unsigned int j,k,off;
	cairo_t* cr;
	cairo_surface_t *surface;
	char msg[80];
	memset(msg, 0x00, sizeof(msg));

	//check custom message
	if (enable_osd && osd_custom_message) {
		std::string filename = "/run/pixelpilot.msg";
		FILE *file = fopen(filename.c_str(), "r");
		osd_tag tag;
		if (file != NULL) {

			if (fgets(custom_msg, sizeof(custom_msg), file) == NULL) {
				perror("Error reading from file");
				fclose(file);
			}
			fclose(file);
			if (unlink(filename.c_str()) != 0) {
				perror("Error deleting the file");
			}
			// Ensure null termination at the 80th position to prevent overflow
			custom_msg[79] = '\0';

			// Find the first newline character, if it exists
			char *newline_pos = strchr(custom_msg, '\n');
			if (newline_pos != NULL) {
				*newline_pos = '\0';  // Null-terminate at the newline
			}
			FactTags fact_tags = { {"file", filename} };
			osd->setFact(Fact(FactMeta("osd.custom_message", fact_tags), std::string(custom_msg)));
			//osd_publish_str_fact("osd.custom_message", &tag, 1, std::string(custom_msg))
			custom_msg_refresh_count = 1;
		}
	}

	int osd_x = buf->width - 300;
	surface = cairo_image_surface_create_for_data(buf->map, CAIRO_FORMAT_ARGB32, buf->width, buf->height, buf->stride);
	cr = cairo_create (surface);

	// https://www.cairographics.org/FAQ/#clear_a_surface
	cairo_save(cr);
	cairo_set_operator(cr, CAIRO_OPERATOR_CLEAR);
	cairo_paint(cr);
	cairo_restore(cr);

	cairo_select_font_face(cr, "Roboto", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_NORMAL);
	cairo_set_font_size(cr, 20);

    if (!video_present.load())
    {
        show_screensaver(cr, buf->width, buf->height, osd->getScreensaverImage());
    }
    if (enable_osd) {
        osd->draw(cr);
    }

	cairo_fill(cr);
	cairo_destroy(cr);
	cairo_surface_destroy(surface);
}

int osd_thread_signal;

typedef struct png_closure
{
	unsigned char * iter;
	unsigned int bytes_left;
} png_closure_t;

cairo_status_t on_read_png_stream(png_closure_t * closure, unsigned char * data, unsigned int length)
{
	if(length > closure->bytes_left) return CAIRO_STATUS_READ_ERROR;
	
	memcpy(data, closure->iter, length);
	closure->iter += length;
	closure->bytes_left -= length;
	return CAIRO_STATUS_SUCCESS;
}

void *__OSD_THREAD__(void *param) {
	osd_thread_params *p = (osd_thread_params *)param;
	Osd *osd = new Osd;
	pthread_setname_np(pthread_self(), "__OSD");

    osd->loadScreensaverImage(p->screensaver_image);
    if (!p->config.empty()) {
	    osd->loadConfig(p->config);
    }
	auto last_display_at = std::chrono::steady_clock::now();

	int ret = pthread_mutex_init(&osd_mutex, NULL);
	assert(!ret);

	struct modeset_buf *buf = &p->out->osd_bufs[p->out->osd_buf_switch];
	ret = modeset_perform_modeset(p->fd, p->out, p->out->osd_request, &p->out->osd_plane,
								  buf->fb, buf->width, buf->height, osd_zpos);
	assert(ret >= 0);
	while (!osd_thread_signal) {
		std::unique_lock<std::mutex> lock(mtx);
		std::vector<Fact> fact_buf;
		auto since_last_display = std::chrono::steady_clock::now() - last_display_at;
		auto wait = std::chrono::milliseconds(refresh_frequency_ms) - since_last_display;
		bool got_fact{false};

        if (enable_osd) {
            got_fact = cv.wait_for(
					lock,
					wait,
					[/*fact_queue*/] {
						return !fact_queue.empty();
					});
        } else {
            sleep(1);
        }
		if (got_fact) {
			// thread woke up because we got a new fact(s)
			// copy all the facts to the temporary buffer to unlock the queue ASAP
			for(; !fact_queue.empty(); fact_queue.pop()) {
				SPDLOG_DEBUG("got fact {}({})", fact_queue.front().getName(), fact_queue.front().getTags());
				fact_buf.push_back(fact_queue.front());
			}
			lock.unlock();
			for (Fact fact : fact_buf) {
				osd->setFact(fact);
			}
			fact_buf.clear();
		} else {
			// thread woke up because of refresh timeout
			lock.unlock();
			SPDLOG_DEBUG("refresh OSD");
			int buf_idx = p->out->osd_buf_switch ^ 1;
			struct modeset_buf *buf = &p->out->osd_bufs[buf_idx];
			modeset_paint_buffer(buf, osd);

			int ret = pthread_mutex_lock(&osd_mutex);
			assert(!ret);	
			p->out->osd_buf_switch = buf_idx;
			ret = pthread_mutex_unlock(&osd_mutex);
			assert(!ret);

			// tell the display thread that we have a update
			ret = pthread_mutex_lock(&video_mutex);
			assert(!ret);
			osd_update_ready = true;
			ret = pthread_cond_signal(&video_cond);
			assert(!ret);
			ret = pthread_mutex_unlock(&video_mutex);
			assert(!ret);

			last_display_at = std::chrono::steady_clock::now();
		}
    }
	spdlog::info("OSD thread done.");
	return nullptr;
}

void mk_tags(osd_tag *tags, int n_tags, FactTags *fact_tags) {
	osd_tag tag;
	for (int i = 0; i < n_tags; i++) {
		tag = *tags++;
		fact_tags->emplace(tag.key, tag.val);
	}
}

void publish(Fact fact) {
	if (!enable_osd) return;
	//SPDLOG_DEBUG("post fact {}({})", fact.getName(), fact.getTags());
	{
		std::lock_guard<std::mutex> lock(mtx);
		fact_queue.push(fact);
	}
	cv.notify_one();
}

#ifdef __cplusplus
extern "C" {
#endif

// Batch APIs

void *osd_batch_init(uint n) {
	auto batch = new std::vector<Fact>;
	batch->reserve(n);
	return batch;
}
void osd_publish_batch(void *batch) {
	std::vector<Fact> *facts = static_cast<std::vector<Fact> *>(batch);
	if (enable_osd) {
		{
			std::lock_guard<std::mutex> lock(mtx);
			for (Fact fact : *facts) {
				// SPDLOG_DEBUG("batch post fact {}({})", fact.getName(), fact.getTags());
				fact_queue.push(fact);
			}
		}
		cv.notify_one();
	}
	delete facts;
};

void osd_add_bool_fact(void *batch, char const *name, osd_tag *tags, int n_tags, bool value) {
	std::vector<Fact> *facts = static_cast<std::vector<Fact> *>(batch);
	FactTags fact_tags;
	mk_tags(tags, n_tags, &fact_tags);
	facts->push_back(Fact(FactMeta(std::string(name), fact_tags), value));
};

void osd_add_int_fact(void *batch, char const *name, osd_tag *tags, int n_tags, long value) {
	std::vector<Fact> *facts = static_cast<std::vector<Fact> *>(batch);
	FactTags fact_tags;
	mk_tags(tags, n_tags, &fact_tags);
	facts->push_back(Fact(FactMeta(std::string(name), fact_tags), value));
};

void osd_add_uint_fact(void *batch, char const *name, osd_tag *tags, int n_tags, ulong value) {
	std::vector<Fact> *facts = static_cast<std::vector<Fact> *>(batch);
	FactTags fact_tags;
	mk_tags(tags, n_tags, &fact_tags);
	facts->push_back(Fact(FactMeta(std::string(name), fact_tags), value));
};

void osd_add_double_fact(void *batch, char const *name, osd_tag *tags, int n_tags, double value) {
	std::vector<Fact> *facts = static_cast<std::vector<Fact> *>(batch);
	FactTags fact_tags;
	mk_tags(tags, n_tags, &fact_tags);
	facts->push_back(Fact(FactMeta(std::string(name), fact_tags), value));
};

void osd_add_str_fact(void *batch, char const *name, osd_tag *tags, int n_tags, const char *value) {
	std::vector<Fact> *facts = static_cast<std::vector<Fact> *>(batch);
	FactTags fact_tags;
	mk_tags(tags, n_tags, &fact_tags);
	facts->push_back(Fact(FactMeta(std::string(name), fact_tags), std::string(value)));
};

void osd_add_clear_fact(void *batch, char const *name, osd_tag *tags, int n_tags) {
	std::vector<Fact> *facts = static_cast<std::vector<Fact> *>(batch);
	FactTags fact_tags;
	mk_tags(tags, n_tags, &fact_tags);
	facts->push_back(Fact(FactMeta(std::string(name), fact_tags)));
}


// Individual APIs

void osd_publish_bool_fact(char const *name, osd_tag *tags, int n_tags, bool value) {
	FactTags fact_tags;
	mk_tags(tags, n_tags, &fact_tags);
	publish(Fact(FactMeta(std::string(name), fact_tags), value));
};

void osd_publish_int_fact(char const *name, osd_tag *tags, int n_tags, long value) {
	FactTags fact_tags;
	mk_tags(tags, n_tags, &fact_tags);
	publish(Fact(FactMeta(std::string(name), fact_tags), value));
};

void osd_publish_uint_fact(char const *name, osd_tag *tags, int n_tags, ulong value) {
	FactTags fact_tags;
	mk_tags(tags, n_tags, &fact_tags);
	publish(Fact(FactMeta(std::string(name), fact_tags), value));
};

void osd_publish_double_fact(char const *name, osd_tag *tags, int n_tags, double value) {
	FactTags fact_tags;
	mk_tags(tags, n_tags, &fact_tags);
	publish(Fact(FactMeta(std::string(name), fact_tags), value));
};

void osd_publish_str_fact(char const *name, osd_tag *tags, int n_tags, const char *value) {
	FactTags fact_tags;
	mk_tags(tags, n_tags, &fact_tags);
	publish(Fact(FactMeta(std::string(name), fact_tags), std::string(value)));
};

void osd_clear_fact(char const *name, osd_tag *tags, int n_tags) {
	FactTags fact_tags;
	mk_tags(tags, n_tags, &fact_tags);
	publish(Fact(FactMeta(std::string(name), fact_tags)));
}

#ifdef __cplusplus
}
#endif
