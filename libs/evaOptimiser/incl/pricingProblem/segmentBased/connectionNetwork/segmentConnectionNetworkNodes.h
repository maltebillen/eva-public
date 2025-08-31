#ifndef EVA_SEGMENT_CONNECTION_NETWORK_NODES_H
#define EVA_SEGMENT_CONNECTION_NETWORK_NODES_H

#include "evaConstants.h"
#include "../segments/segment.h"

namespace eva
{
	namespace sbn
	{
		namespace con
		{
			enum class ConNodeType : uint8_t
			{
				SEGMENT = 1,
				START_SCHEDULE,
				FIXED_CHARGING,
				END_SCHEDULE,
				UNDEFINED
			};

			class ConBaseNodeData
			{
			public:
				// INLINE FUNCTIONS
				inline std::unique_ptr<ConBaseNodeData> clone() const { return std::unique_ptr<ConBaseNodeData>(deepClone()); };

				virtual inline const Types::Index get_startChargerIndex() const = 0;
				virtual inline const Types::Index get_endChargerIndex() const = 0;
				virtual inline const Types::DateTime get_startTime() const = 0;
				virtual inline const Types::DateTime get_endTime() const = 0;

				virtual inline const uint32_t get_minlb_rechargeDuration() const = 0;
				virtual inline const uint32_t get_maxub_rechargeDuration() const = 0;

				virtual inline const uint32_t get_updated_distance(const uint32_t &prev_distance) const = 0;
				virtual inline const uint32_t get_distanceBeforeMaintenance() const = 0;
				virtual inline const uint32_t get_distanceAfterMaintenance() const = 0;
				virtual inline const double get_cost(const uint32_t &prev_distance) const = 0;
				virtual inline const double get_accDuals() const = 0;

				virtual ~ConBaseNodeData() {};

			protected:
				virtual inline ConBaseNodeData *deepClone() const = 0;
			};

			class UndefinedNodeData : public ConBaseNodeData
			{
			public:
				// CONSTRUCTORS
				UndefinedNodeData() {};
				~UndefinedNodeData() {};

				// GETTERS

				virtual inline const Types::Index get_startChargerIndex() const override { return Constants::BIG_INDEX; };
				virtual inline const Types::Index get_endChargerIndex() const override { return Constants::BIG_INDEX; };
				virtual inline const Types::DateTime get_startTime() const override { return Constants::MAX_TIMESTAMP; };
				virtual inline const Types::DateTime get_endTime() const override { return Constants::MAX_TIMESTAMP; };

				virtual inline const uint32_t get_minlb_rechargeDuration() const override { return 0; };
				virtual inline const uint32_t get_maxub_rechargeDuration() const override { return 0; };

				virtual inline const uint32_t get_updated_distance(const uint32_t &prev_distance) const override { return prev_distance; };
				virtual inline const uint32_t get_distanceBeforeMaintenance() const override { return 0; };
				virtual inline const uint32_t get_distanceAfterMaintenance() const override { return 0; };
				virtual inline const double get_cost(const uint32_t &prev_distance) const override { return 0.0; };
				virtual inline const double get_accDuals() const override { return 0.0; };

			protected:
				virtual inline UndefinedNodeData *deepClone() const override { return new UndefinedNodeData(*this); };
			};

			class SegmentPieceNodeData : public ConBaseNodeData
			{
				// ATTRIBUTES

				const Segment &_segment;
				const subgraph::NonDominatedSchedulePiece &_nonDominatedSchedulePiece;

			public:
				// CONSTRUCTORS

				SegmentPieceNodeData(
					const Segment &segment,
					const subgraph::NonDominatedSchedulePiece &nonDominatedSchedulePiece) : _segment(segment),
																							_nonDominatedSchedulePiece(nonDominatedSchedulePiece)
				{
				}

				~SegmentPieceNodeData() {};
				// GETTERS

				virtual inline const Types::Index get_startChargerIndex() const override { return _segment.get_startCharger().get_index(); };
				virtual inline const Types::Index get_endChargerIndex() const override { return _segment.get_endCharger().get_index(); };
				virtual inline const Types::DateTime get_startTime() const override { return _nonDominatedSchedulePiece.get_startTime(); };
				virtual inline const Types::DateTime get_endTime() const override { return _nonDominatedSchedulePiece.get_endTime(); };

				virtual inline const uint32_t get_minlb_rechargeDuration() const override { return _segment.get_minlb_rechargeDuration(); };
				virtual inline const uint32_t get_maxub_rechargeDuration() const override { return _segment.get_maxub_rechargeDuration(); };

				virtual inline const uint32_t get_updated_distance(const uint32_t &prev_distance) const override { return _segment.get_updated_distance(prev_distance); };
				virtual inline const uint32_t get_distanceBeforeMaintenance() const override { return _segment.get_distanceBeforeMaintenance(); };
				virtual inline const uint32_t get_distanceAfterMaintenance() const override { return _segment.get_distanceAfterMaintenance(); };
				virtual inline const double get_cost(const uint32_t &prev_distance) const override { return _segment.get_cost(prev_distance); };
				virtual inline const double get_accDuals() const override { return _nonDominatedSchedulePiece.get_accDuals(); };

				// LOCAL GETTERS:

				inline const Segment &get_segment() const { return _segment; };
				inline const subgraph::NonDominatedSchedulePiece &get_nonDominatedSchedulePiece() const { return _nonDominatedSchedulePiece; };

			protected:
				virtual inline SegmentPieceNodeData *deepClone() const override { return new SegmentPieceNodeData(*this); };
			};

			class StartScheduleNodeData : public ConBaseNodeData
			{
				const Types::Index _indexCharger;
				const Types::DateTime _time;

			public:
				// CONSTRUCTORS

				StartScheduleNodeData(
					const Charger &charger,
					const Types::DateTime &time) : _indexCharger(charger.get_index()),
												   _time(time)
				{
				}

				StartScheduleNodeData(
					const Types::Index &indexCharger,
					const Types::DateTime &time) : _indexCharger(indexCharger),
												   _time(time)
				{
				}

				~StartScheduleNodeData() {};
				// GETTERS

				virtual inline const Types::Index get_startChargerIndex() const override { return _indexCharger; };
				virtual inline const Types::Index get_endChargerIndex() const override { return _indexCharger; };
				virtual inline const Types::DateTime get_startTime() const override { return _time; };
				virtual inline const Types::DateTime get_endTime() const override { return _time; };

				virtual inline const uint32_t get_minlb_rechargeDuration() const override { return 0; };
				virtual inline const uint32_t get_maxub_rechargeDuration() const override { return 0; };

				virtual inline const uint32_t get_updated_distance(const uint32_t &prev_distance) const override { return prev_distance; };
				virtual inline const uint32_t get_distanceBeforeMaintenance() const override { return 0; };
				virtual inline const uint32_t get_distanceAfterMaintenance() const override { return 0; };
				virtual inline const double get_cost(const uint32_t &prev_distance) const override { return 0.0; };
				virtual inline const double get_accDuals() const override { return 0.0; };

			protected:
				virtual inline StartScheduleNodeData *deepClone() const override { return new StartScheduleNodeData(*this); };
			};

			class FixedChargingNodeData : public ConBaseNodeData
			{
				const Types::Index _index_fixed_charger = Constants::BIG_INDEX;
				const Types::Index _index_fixed_vehicle = Constants::BIG_INDEX;
				const Types::Index _index_fixed_from_schedule_node = Constants::BIG_INDEX;
				const Types::Index _index_fixed_to_schedule_node = Constants::BIG_INDEX;
				const Types::DateTime _fixed_start_time;

			public:
				// CONSTRUCTORS
				
				FixedChargingNodeData(
					const Types::Index &index_fixed_charger,
					const Types::Index &index_fixed_vehicle,
					const Types::Index &index_fixed_from_schedule_node,
					const Types::Index &index_fixed_to_schedule_node,
					const Types::DateTime &fixed_start_time
				) :
					_index_fixed_charger(index_fixed_charger),
					_index_fixed_vehicle(index_fixed_vehicle),
					_index_fixed_from_schedule_node(index_fixed_from_schedule_node),
					_index_fixed_to_schedule_node(index_fixed_to_schedule_node),
					_fixed_start_time(fixed_start_time)
				{};

				~FixedChargingNodeData() {};

				// CUSTOM GETTERS:
				inline const Types::Index& get_index_fixed_vehicle() const { return _index_fixed_vehicle;};
				inline const Types::Index& get_index_fixed_from_schedule_node() const { return _index_fixed_from_schedule_node;};
				inline const Types::Index& get_index_fixed_to_schedule_node() const { return _index_fixed_to_schedule_node;};

				// GETTERS

				virtual inline const Types::Index get_startChargerIndex() const override { return _index_fixed_charger; };
				virtual inline const Types::Index get_endChargerIndex() const override { return _index_fixed_charger; };
				virtual inline const Types::DateTime get_startTime() const override { return _fixed_start_time; };
				virtual inline const Types::DateTime get_endTime() const override { return _fixed_start_time; };

				virtual inline const uint32_t get_minlb_rechargeDuration() const override { return 0; };
				virtual inline const uint32_t get_maxub_rechargeDuration() const override { return 0; };

				virtual inline const uint32_t get_updated_distance(const uint32_t &prev_distance) const override { return prev_distance; };
				virtual inline const uint32_t get_distanceBeforeMaintenance() const override { return 0; };
				virtual inline const uint32_t get_distanceAfterMaintenance() const override { return 0; };
				virtual inline const double get_cost(const uint32_t &prev_distance) const override { return 0.0; };
				virtual inline const double get_accDuals() const override { return 0.0; };

			protected:
				virtual inline FixedChargingNodeData *deepClone() const override { return new FixedChargingNodeData(*this); };
			};

			class EndScheduleNodeData : public ConBaseNodeData
			{

			public:
				// CONSTRUCTORS

				EndScheduleNodeData() {};
				~EndScheduleNodeData() {};

				// GETTERS

				virtual inline const Types::Index get_startChargerIndex() const override { return Constants::BIG_INDEX; };
				virtual inline const Types::Index get_endChargerIndex() const override { return Constants::BIG_INDEX; };
				virtual inline const Types::DateTime get_startTime() const override { return Constants::MAX_TIMESTAMP; };
				virtual inline const Types::DateTime get_endTime() const override { return Constants::MAX_TIMESTAMP; };

				virtual inline const uint32_t get_minlb_rechargeDuration() const override { return 0; };
				virtual inline const uint32_t get_maxub_rechargeDuration() const override { return 0; };

				virtual inline const uint32_t get_updated_distance(const uint32_t &prev_distance) const override { return prev_distance; };
				virtual inline const uint32_t get_distanceBeforeMaintenance() const override { return 0; };
				virtual inline const uint32_t get_distanceAfterMaintenance() const override { return 0; };
				virtual inline const double get_cost(const uint32_t &prev_distance) const override { return 0.0; };
				virtual inline const double get_accDuals() const override { return 0.0; };

			protected:
				virtual inline EndScheduleNodeData *deepClone() const override { return new EndScheduleNodeData(*this); };
			};

			class ConNodeData
			{
				std::vector<Types::AccessType> _vecAccess;
				Types::Index _index_fixed_vehicle = Constants::BIG_INDEX;
				Types::DateTime _max_rc_start_time = 0;
				bool _flag_outgoing_to_sink_allowed = true;

			public:
				std::unique_ptr<ConBaseNodeData> ptrNodeData;
				Types::Index index = Constants::BIG_INDEX;
				ConNodeType type = ConNodeType::UNDEFINED;

				// CONSTRUCTORS

				ConNodeData() : index(Constants::BIG_INDEX),
										 type(ConNodeType::UNDEFINED),
										 ptrNodeData(std::make_unique<UndefinedNodeData>()) {};

				ConNodeData(
					const SegmentPieceNodeData &spnd,
					const uint32_t &nrVehicles) : type(ConNodeType::SEGMENT),
												  ptrNodeData(std::make_unique<SegmentPieceNodeData>(spnd))
				{
					_vecAccess.resize(nrVehicles, Types::AccessType::ALLOWED);
				};

				ConNodeData(
					const StartScheduleNodeData &vsnd,
					const uint32_t &nrVehicles) : type(ConNodeType::START_SCHEDULE),
												  ptrNodeData(std::make_unique<StartScheduleNodeData>(vsnd))
				{
					_vecAccess.resize(nrVehicles, Types::AccessType::ALLOWED);
				};

				ConNodeData(
					const FixedChargingNodeData& fcnd,
					const uint32_t &nrVehicles) : type(ConNodeType::FIXED_CHARGING),
												  ptrNodeData(std::make_unique<FixedChargingNodeData>(fcnd))
				{
					_vecAccess.resize(nrVehicles, Types::AccessType::ALLOWED);
				};

				ConNodeData(
					const EndScheduleNodeData &vend,
					const uint32_t &nrVehicles) : type(ConNodeType::END_SCHEDULE),
												  ptrNodeData(std::make_unique<EndScheduleNodeData>(vend))
				{
					_vecAccess.resize(nrVehicles, Types::AccessType::ALLOWED);
				};

				// COPY CONSTRUCTORS:

				ConNodeData(ConNodeData const &other) : index(other.index),
																		  type(other.type),
																		  ptrNodeData(other.ptrNodeData->clone()),
																		  _vecAccess(other._vecAccess),
																		  _index_fixed_vehicle(other._index_fixed_vehicle),
																		  _max_rc_start_time(other._max_rc_start_time),
																		  _flag_outgoing_to_sink_allowed(other._flag_outgoing_to_sink_allowed)
				{
				}
				ConNodeData(ConNodeData &&other) = default;
				ConNodeData &operator=(ConNodeData const &other)
				{
					ptrNodeData = other.ptrNodeData->clone();
					index = other.index;
					type = other.type;
					_vecAccess = other._vecAccess;
					_index_fixed_vehicle = other._index_fixed_vehicle;
					_max_rc_start_time = other._max_rc_start_time;
					_flag_outgoing_to_sink_allowed = other._flag_outgoing_to_sink_allowed;
					return *this;
				}
				ConNodeData &operator=(ConNodeData &&other) = default;

				// DESTRUCTOR:

				~ConNodeData() = default;

				// GETTERS:

				inline const Types::Index& get_index() const { return index; };
				inline const ConNodeType get_type() const { return type; };

				inline const Types::Index get_startChargerIndex() const { return ptrNodeData->get_startChargerIndex(); };
				inline const Types::Index get_endChargerIndex() const { return ptrNodeData->get_endChargerIndex(); };
				inline const Types::DateTime get_startTime() const { return ptrNodeData->get_startTime(); };
				inline const Types::DateTime get_endTime() const { return ptrNodeData->get_endTime(); };
				inline const uint32_t get_minlb_rechargeDuration() const { return ptrNodeData->get_minlb_rechargeDuration(); };
				inline const uint32_t get_maxub_rechargeDuration() const { return ptrNodeData->get_maxub_rechargeDuration(); };

				inline const uint32_t get_updated_distance(const uint32_t &prev_distance) const { return ptrNodeData->get_updated_distance(prev_distance); };
				inline const uint32_t get_distanceBeforeMaintenance() const { return ptrNodeData->get_distanceBeforeMaintenance(); };
				inline const uint32_t get_distanceAfterMaintenance() const { return ptrNodeData->get_distanceAfterMaintenance(); };
				inline const uint32_t get_distance() const { return ptrNodeData->get_distanceBeforeMaintenance() + ptrNodeData->get_distanceAfterMaintenance(); };
				inline const double get_cost(const uint32_t &prev_distance) const { return ptrNodeData->get_cost(prev_distance); };
				inline const double get_accDuals() const { return ptrNodeData->get_accDuals(); };

				inline void init_access(const std::vector<Types::AccessType>& vecDefault) { _vecAccess = vecDefault;}
				inline void set_access_all(const Types::AccessType val_access) { std::fill(_vecAccess.begin(), _vecAccess.end(), val_access);};
				inline void set_access_vehicle(const Types::Index& indexVehicle, const Types::AccessType val_access) { _vecAccess[indexVehicle] = val_access;};
				inline void fix_vehicle(const Types::Index& indexVehicle, const Types::DateTime& max_rc_start_time) { _index_fixed_vehicle = indexVehicle; _max_rc_start_time = max_rc_start_time;};
				inline void set_flag_outgoing_to_sink_allowed(bool flag) { _flag_outgoing_to_sink_allowed = flag;};

				inline const Types::DateTime& get_max_rc_start_time() const { return _max_rc_start_time;};
				inline const Types::Index& get_indexFixedVehicle() const { return _index_fixed_vehicle;};
				inline const bool get_has_fixed_activity(const Types::Index& indexVehicle) const { return _index_fixed_vehicle == indexVehicle;}
				inline const std::vector<Types::AccessType>& get_vecAccess() const { return _vecAccess;};
				inline const bool has_access(const Types::Index &indexVehicle) const { return _vecAccess[indexVehicle] == Types::AccessType::ALLOWED; };
				inline const bool is_flag_outgoing_to_sink_allowed() const { return _flag_outgoing_to_sink_allowed;};

				inline const SegmentPieceNodeData *castSegmentPieceNodeData() const { return static_cast<const SegmentPieceNodeData *>(ptrNodeData.get()); };
				inline const StartScheduleNodeData *castStartScheduleNodeData() const { return static_cast<const StartScheduleNodeData *>(ptrNodeData.get()); };
				inline const FixedChargingNodeData *castFixedChargingNodeData() const { return static_cast<const FixedChargingNodeData *>(ptrNodeData.get()); };
				inline const EndScheduleNodeData *castEndScheduleNodeData() const { return static_cast<const EndScheduleNodeData *>(ptrNodeData.get()); }
			};
		}

	}
}

#endif // !EVA_SEGMENT_CONNECTION_NETWORK_NODES_H
